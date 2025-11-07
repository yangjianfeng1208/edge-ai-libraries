from fastapi import APIRouter, WebSocket, WebSocketDisconnect, status

import logging
from typing import Set, Optional
from asyncio import Lock

router = APIRouter()
logger = logging.getLogger("api.routes.metrics")

# Single collector websocket (None if not connected)
collector_ws: Optional[WebSocket] = None
collector_lock = Lock()

# Set of client websockets
client_connections: Set[WebSocket] = set()
clients_lock = Lock()


@router.websocket("/ws/collector")
async def collector_websocket(websocket: WebSocket):
    """
    WebSocket endpoint for the collector to push metrics to the server.
    Only one collector allowed at a time.
    Each received message (list of metrics) is broadcast to all connected clients.
    """
    global collector_ws
    await websocket.accept()
    logger.debug("Collector connected from %s", websocket.client)
    async with collector_lock:
        if collector_ws is not None:
            logger.warning("Rejecting new collector: one is already connected.")
            # Optionally, send a message then close
            await websocket.send_text("Collector already connected; only one allowed.")
            await websocket.close(code=status.WS_1008_POLICY_VIOLATION)
            return
        collector_ws = websocket
    try:
        while True:
            data = await websocket.receive_json(mode="binary")
            logger.debug("Received metrics from collector: %s", data)

            # Broadcast to all clients
            disconnects = []
            async with clients_lock:
                clients = list(client_connections)
            for client in clients:
                try:
                    await client.send_json(data, mode="text")
                    logger.debug("Forwarded metrics to client %s", client.client)
                except Exception as e:
                    logger.error("Error sending to client %s: %s", client.client, e)
                    disconnects.append(client)
            # Cleanup disconnected clients
            if disconnects:
                async with clients_lock:
                    for client in disconnects:
                        client_connections.discard(client)
                logger.debug("Cleaned up %d disconnected clients", len(disconnects))
    except WebSocketDisconnect:
        logger.info("Collector disconnected: %s", websocket.client)
    except Exception as e:
        logger.error("Exception in collector handler: %s", e)
    finally:
        async with collector_lock:
            if collector_ws == websocket:
                collector_ws = None
        logger.debug("Collector slot released")


@router.websocket("/ws/clients")
async def clients_websocket(websocket: WebSocket):
    """
    WebSocket endpoint for clients that receive metrics in real time.
    """
    await websocket.accept()
    logger.debug("Client connected: %s", websocket.client)
    async with clients_lock:
        client_connections.add(websocket)
    try:
        while True:
            # Either wait for ping/pong, or just sleep to keep connection open.
            msg = await websocket.receive_text()
            logger.debug("Received message from client (ignored): %s", msg)
    except WebSocketDisconnect:
        logger.debug("Client disconnected: %s", websocket.client)
    except Exception as e:
        logger.error("Exception in client handler: %s", e)
    finally:
        async with clients_lock:
            client_connections.discard(websocket)

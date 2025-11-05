import unittest
from fastapi import FastAPI
from fastapi.testclient import TestClient

from api.routes.metrics import router as metrics_router


class TestMetricsAPI(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Set up test client once for all tests."""
        app = FastAPI()
        app.include_router(metrics_router, prefix="/metrics")
        cls.client = TestClient(app)

    def test_placeholder(self):
        """Placeholder test to ensure the test suite runs."""
        # Test WebSocket endpoint exists (404 for regular GET is expected)
        response = self.client.get("/metrics")
        self.assertEqual(response.status_code, 404)

    def test_websocket_collector_endpoint(self):
        """Test that the collector WebSocket endpoint can be connected to."""
        with self.client.websocket_connect("/metrics/ws/collector") as websocket:
            # Connection successful
            self.assertIsNotNone(websocket)

    def test_websocket_clients_endpoint(self):
        """Test that the clients WebSocket endpoint can be connected to."""
        with self.client.websocket_connect("/metrics/ws/clients") as websocket:
            # Connection successful
            self.assertIsNotNone(websocket)

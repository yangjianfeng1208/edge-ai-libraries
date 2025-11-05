#!/bin/bash

# Run video manager and exit if it fails
if ! ./video_manager.sh; then
    echo "[ERROR] video_manager.sh failed. Exiting."
    exit 1
fi

fastapi run --port 7860 api/main.py

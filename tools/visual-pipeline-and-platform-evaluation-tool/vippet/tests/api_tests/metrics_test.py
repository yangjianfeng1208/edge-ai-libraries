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
        response = self.client.get("/metrics")
        self.assertIn(response.status_code, [200, 400])

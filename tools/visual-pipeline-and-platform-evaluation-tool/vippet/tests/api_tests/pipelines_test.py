import unittest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock

from api.routes.pipelines import router as pipelines_router


class TestPipelinesAPI(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Set up test client once for all tests."""
        app = FastAPI()
        app.include_router(pipelines_router, prefix="/pipelines")
        cls.client = TestClient(app)

    @patch("api.routes.pipelines.gst_inspector")
    @patch("api.routes.pipelines.PipelineLoader")
    def test_get_pipelines_returns_list(self, mock_pipeline_loader, mock_gst_inspector):
        mock_pipeline = MagicMock()
        mock_pipeline.get_default_gst_launch.return_value = (
            "gst-launch-1.0 filesrc location=<file_path> ! decodebin !"
        )
        mock_gst_inspector.get_elements.return_value = ["vapostproc", "vacompositor"]
        mock_pipeline_loader.list.return_value = ["pipeline1"]
        mock_pipeline_loader.load.return_value = (
            mock_pipeline,
            {
                "name": "Test Pipeline",
                "version": "1.0.0",
                "definition": "A test pipeline",
            },
        )

        response = self.client.get("/pipelines")
        assert response.status_code == 200
        data = response.json()
        assert isinstance(data, list)

        # Check the contents of the first pipeline
        assert data[0]["name"] == "Test Pipeline"
        assert data[0]["version"] == "1.0.0"
        assert data[0]["description"] == "A test pipeline"
        assert (
            data[0]["parameters"]["default"]["launch_string"]
            == "gst-launch-1.0 filesrc location=<file_path> ! decodebin !"
        )

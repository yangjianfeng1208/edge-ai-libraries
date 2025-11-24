import unittest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from unittest.mock import patch

import api.api_schemas as schemas
from api.routes.pipelines import router as pipelines_router


class TestPipelinesAPI(unittest.TestCase):
    test_graph = """
    {
        "nodes": [
            {
                "id": "0",
                "type": "filesrc",
                "data": {
                    "location": "/tmp/license-plate-detection.mp4"
                }
            },
            {
                "id": "1",
                "type": "autovideosink",
                "data": {}
            }
        ],
        "edges": [
            {
                "id": "0",
                "source": "0",
                "target": "1"
            }
        ]
    }
    """

    @classmethod
    def setUpClass(cls):
        """Set up test client once for all tests."""
        app = FastAPI()
        app.include_router(pipelines_router, prefix="/pipelines")
        cls.client = TestClient(app)

    @patch("api.routes.pipelines.pipeline_manager")
    def test_get_pipelines_returns_list(self, mock_pipeline_manager):
        mock_pipeline_manager.get_pipelines.return_value = [
            schemas.Pipeline(
                id="pipeline-abc123",
                name="predefined-pipelines",
                version=1,
                description="Smart Network Video Recorder (NVR) Proxy Pipeline",
                source=schemas.PipelineSource.PREDEFINED,
                type=schemas.PipelineType.GSTREAMER,
                pipeline_graph=schemas.PipelineGraph.model_validate_json(
                    self.test_graph
                ),
                parameters=None,
            ),
            schemas.Pipeline(
                id="pipeline-def456",
                name="user-defined-pipelines",
                version=1,
                description="Test Pipeline Description",
                source=schemas.PipelineSource.USER_CREATED,
                type=schemas.PipelineType.GSTREAMER,
                pipeline_graph=schemas.PipelineGraph.model_validate_json(
                    self.test_graph
                ),
                parameters=None,
            ),
        ]

        response = self.client.get("/pipelines")

        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertIsInstance(data, list)
        self.assertEqual(len(data), 2)

        # Check the contents of the first pipeline
        first_pipeline = data[0]
        self.assertEqual(first_pipeline["id"], "pipeline-abc123")
        self.assertEqual(first_pipeline["name"], "predefined-pipelines")
        self.assertEqual(first_pipeline["version"], 1)
        self.assertEqual(
            first_pipeline["description"],
            "Smart Network Video Recorder (NVR) Proxy Pipeline",
        )
        self.assertEqual(first_pipeline["type"], schemas.PipelineType.GSTREAMER)
        self.assertIn("pipeline_graph", first_pipeline)
        self.assertIsNone(first_pipeline["parameters"])

        # Check the contents of the second pipeline
        second_pipeline = data[1]
        self.assertEqual(second_pipeline["id"], "pipeline-def456")
        self.assertEqual(second_pipeline["name"], "user-defined-pipelines")
        self.assertEqual(second_pipeline["version"], 1)
        self.assertEqual(second_pipeline["description"], "Test Pipeline Description")
        self.assertEqual(second_pipeline["type"], schemas.PipelineType.GSTREAMER)
        self.assertIn("pipeline_graph", second_pipeline)
        self.assertIsNone(second_pipeline["parameters"])

    @patch("api.routes.pipelines.pipeline_manager")
    def test_create_pipeline_valid(self, mock_pipeline_manager):
        # Mock the return value to include the pipeline with ID
        mock_pipeline = schemas.Pipeline(
            id="pipeline-newtest",
            name="user-defined-pipelines",
            version=1,
            description="A custom test pipeline",
            source=schemas.PipelineSource.USER_CREATED,
            type=schemas.PipelineType.GSTREAMER,
            pipeline_graph=schemas.PipelineGraph.model_validate_json(self.test_graph),
            parameters=None,
        )
        mock_pipeline_manager.add_pipeline.return_value = mock_pipeline

        new_pipeline = {
            "name": "user-defined-pipelines",
            "version": 1,
            "description": "A custom test pipeline",
            "type": schemas.PipelineType.GSTREAMER,
            "pipeline_description": "filesrc location=/tmp/test.mp4 ! decodebin ! autovideosink",
            "parameters": None,
        }

        response = self.client.post("/pipelines", json=new_pipeline)

        self.assertEqual(response.status_code, 201)
        self.assertIn("Location", response.headers)
        self.assertEqual(
            response.headers["Location"],
            "/pipelines/pipeline-newtest",
        )
        self.assertEqual(response.json(), {"message": "Pipeline created"})

    @patch("api.routes.pipelines.pipeline_manager")
    def test_create_pipeline_duplicate(self, mock_pipeline_manager):
        mock_pipeline_manager.add_pipeline.side_effect = ValueError(
            "Pipeline already exists."
        )

        duplicate_pipeline = {
            "name": "user-defined-pipelines",
            "version": 1,
            "description": "A custom test pipeline",
            "type": schemas.PipelineType.GSTREAMER,
            "pipeline_description": "filesrc location=/tmp/test.mp4 ! decodebin ! autovideosink",
            "parameters": None,
        }

        response = self.client.post("/pipelines", json=duplicate_pipeline)

        self.assertEqual(response.status_code, 400)
        self.assertEqual(
            response.json(),
            schemas.MessageResponse(message="Pipeline already exists.").model_dump(),
        )

    @patch("api.routes.pipelines.pipeline_manager")
    def test_create_pipeline_server_error(self, mock_pipeline_manager):
        mock_pipeline_manager.add_pipeline.side_effect = Exception("Unexpected error")

        new_pipeline = {
            "name": "user-defined-pipelines",
            "version": 1,
            "description": "A custom test pipeline",
            "type": schemas.PipelineType.GSTREAMER,
            "pipeline_description": "filesrc location=/tmp/test.mp4 ! decodebin ! autovideosink",
            "parameters": None,
        }

        response = self.client.post("/pipelines", json=new_pipeline)

        self.assertEqual(response.status_code, 500)
        self.assertEqual(
            response.json(),
            schemas.MessageResponse(
                message="Failed to create pipeline: Unexpected error"
            ).model_dump(),
        )

    @patch("api.routes.pipelines.pipeline_manager")
    def test_get_pipeline_by_id_found(self, mock_pipeline_manager):
        mock_pipeline_manager.get_pipeline_by_id.return_value = schemas.Pipeline(
            id="pipeline-ghi789",
            name="user-defined-pipelines",
            version=1,
            description="A custom test pipeline",
            source=schemas.PipelineSource.USER_CREATED,
            type=schemas.PipelineType.GSTREAMER,
            pipeline_graph=schemas.PipelineGraph.model_validate_json(self.test_graph),
            parameters=None,
        )

        response = self.client.get("/pipelines/pipeline-ghi789")

        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertEqual(data["id"], "pipeline-ghi789")
        self.assertEqual(data["name"], "user-defined-pipelines")
        self.assertEqual(data["version"], 1)
        self.assertEqual(data["description"], "A custom test pipeline")
        self.assertEqual(data["type"], schemas.PipelineType.GSTREAMER)
        self.assertIn("pipeline_graph", data)
        self.assertIsNone(data["parameters"])

    @patch("api.routes.pipelines.pipeline_manager")
    def test_get_pipeline_by_id_not_found(self, mock_pipeline_manager):
        mock_pipeline_manager.get_pipeline_by_id.side_effect = ValueError(
            "Pipeline with id 'nonexistent-id' not found."
        )

        response = self.client.get("/pipelines/nonexistent-id")

        self.assertEqual(response.status_code, 404)
        self.assertEqual(
            response.json(),
            schemas.MessageResponse(
                message="Pipeline with id 'nonexistent-id' not found."
            ).model_dump(),
        )

    @patch("api.routes.pipelines.pipeline_manager")
    def test_get_pipeline_by_id_server_error(self, mock_pipeline_manager):
        mock_pipeline_manager.get_pipeline_by_id.side_effect = Exception(
            "Unexpected error"
        )

        response = self.client.get("/pipelines/pipeline-test123")

        self.assertEqual(response.status_code, 500)
        self.assertEqual(
            response.json(),
            schemas.MessageResponse(
                message="Unexpected error: Unexpected error"
            ).model_dump(),
        )

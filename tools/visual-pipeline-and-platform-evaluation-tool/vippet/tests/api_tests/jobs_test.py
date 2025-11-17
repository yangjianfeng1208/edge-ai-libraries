import unittest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from fastapi.routing import APIRoute
from unittest.mock import patch

import api.api_schemas as schemas
from api.routes.jobs import router as jobs_router


class TestJobsAPI(unittest.TestCase):
    """
    Integration-style unit tests for the optimization jobs HTTP API.

    The tests use FastAPI's TestClient and patch the global
    ``optimization_manager`` object exposed by ``api.routes.jobs`` so we can
    precisely control the behavior of the underlying manager without
    touching its real implementation or any background threads.

    The overall design mirrors tests/api_tests/pipelines_test.py:

    * we mount only the router we want to test on a lightweight FastAPI app,
    * we exercise the real path configuration and response models,
    * we always validate both HTTP status codes and JSON payloads.
    """

    @classmethod
    def setUpClass(cls):
        """
        Build a minimal FastAPI app and mount the jobs router once for all tests.

        This mirrors the approach used in ``pipelines_test.py`` in order to:
        * exercise the actual path/operation configuration of the router,
        * verify serialization / response models and HTTP codes,
        * keep the tests fast and side-effect free by patching dependencies.
        """
        app = FastAPI()
        # All endpoints in jobs.py are mounted under the /jobs prefix.
        # This prefix is baked into all request URLs used in this test suite.
        app.include_router(jobs_router, prefix="/jobs")
        cls.client = TestClient(app)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _make_minimal_graph(self) -> schemas.PipelineGraph:
        """
        Build a very small pipeline graph to be used in job status payloads.

        Only the shape of the object matters for the tests; the actual
        content of nodes and edges is irrelevant as long as it is valid.
        """
        return schemas.PipelineGraph(
            nodes=[
                schemas.Node(
                    id="0",
                    type="filesrc",
                    data={"location": "/tmp/dummy.mp4"},
                )
            ],
            edges=[],
        )

    # ------------------------------------------------------------------
    # /jobs/optimization/status
    # ------------------------------------------------------------------

    @patch("api.routes.jobs.optimization_manager")
    def test_get_optimization_statuses_returns_list(self, mock_optimization_manager):
        """
        The /jobs/optimization/status endpoint should return a list of
        OptimizationJobStatus objects as JSON.

        This test validates:
        * HTTP 200 status,
        * response shape (list of objects),
        * selected field values are correctly serialized.
        """
        graph = self._make_minimal_graph()

        # Simulate two existing jobs with different types and states.
        mock_optimization_manager.get_all_job_statuses.return_value = [
            schemas.OptimizationJobStatus(
                id="job-1",
                type=schemas.OptimizationType.PREPROCESS,
                start_time=1000,
                elapsed_time=200,
                state=schemas.OptimizationJobState.RUNNING,
                total_fps=None,
                original_pipeline_graph=graph,
                optimized_pipeline_graph=None,
                original_pipeline_description="filesrc ! decodebin ! sink",
                optimized_pipeline_description=None,
                error_message=None,
            ),
            schemas.OptimizationJobStatus(
                id="job-2",
                type=schemas.OptimizationType.OPTIMIZE,
                start_time=2000,
                elapsed_time=500,
                state=schemas.OptimizationJobState.COMPLETED,
                total_fps=123.4,
                original_pipeline_graph=graph,
                optimized_pipeline_graph=graph,
                original_pipeline_description="filesrc ! decodebin ! sink",
                optimized_pipeline_description="optimized-pipeline",
                error_message=None,
            ),
        ]

        response = self.client.get("/jobs/optimization/status")

        self.assertEqual(response.status_code, 200)
        data = response.json()

        # Basic shape: list with two entries
        self.assertIsInstance(data, list)
        self.assertEqual(len(data), 2)

        first, second = data[0], data[1]

        # Spot-check first job
        self.assertEqual(first["id"], "job-1")
        self.assertEqual(first["type"], schemas.OptimizationType.PREPROCESS)
        self.assertEqual(first["state"], schemas.OptimizationJobState.RUNNING)
        self.assertIsNone(first["total_fps"])
        self.assertIn("original_pipeline_graph", first)
        self.assertIsNone(first["optimized_pipeline_graph"])

        # Spot-check second job
        self.assertEqual(second["id"], "job-2")
        self.assertEqual(second["type"], schemas.OptimizationType.OPTIMIZE)
        self.assertEqual(second["state"], schemas.OptimizationJobState.COMPLETED)
        self.assertEqual(second["total_fps"], 123.4)
        self.assertEqual(second["optimized_pipeline_description"], "optimized-pipeline")

    # ------------------------------------------------------------------
    # /jobs/optimization/{job_id}
    # ------------------------------------------------------------------

    @patch("api.routes.jobs.optimization_manager")
    def test_get_optimization_job_summary_found(self, mock_optimization_manager):
        """
        When the manager returns an OptimizationJobSummary, the endpoint
        must respond with HTTP 200 and the serialized summary.

        This exercises the "happy path" where a job with the given id
        exists and demonstrates that the original request object is
        correctly embedded in the response.
        """
        request = schemas.PipelineRequestOptimize(
            type=schemas.OptimizationType.PREPROCESS,
            parameters={"foo": "bar"},
        )
        mock_optimization_manager.get_job_summary.return_value = (
            schemas.OptimizationJobSummary(
                id="job-123",
                request=request,
            )
        )

        response = self.client.get("/jobs/optimization/job-123")

        self.assertEqual(response.status_code, 200)
        data = response.json()

        self.assertEqual(data["id"], "job-123")
        # Ensure the request part of the summary is present and correct
        self.assertIn("request", data)
        self.assertEqual(data["request"]["type"], schemas.OptimizationType.PREPROCESS)
        self.assertEqual(data["request"]["parameters"], {"foo": "bar"})

        # Manager should have been called with the same job id
        mock_optimization_manager.get_job_summary.assert_called_once_with("job-123")

    @patch("api.routes.jobs.optimization_manager")
    def test_get_optimization_job_summary_not_found(self, mock_optimization_manager):
        """
        When the manager returns None, the endpoint should return a 404
        with a descriptive MessageResponse payload.
        """
        mock_optimization_manager.get_job_summary.return_value = None

        missing_job_id = "missing-job"
        response = self.client.get(f"/jobs/optimization/{missing_job_id}")

        self.assertEqual(response.status_code, 404)
        self.assertEqual(
            response.json(),
            schemas.MessageResponse(
                message=f"Optimization job {missing_job_id} not found"
            ).model_dump(),
        )

        mock_optimization_manager.get_job_summary.assert_called_once_with(
            missing_job_id
        )

    # ------------------------------------------------------------------
    # /jobs/optimization/{job_id}/status
    # ------------------------------------------------------------------

    @patch("api.routes.jobs.optimization_manager")
    def test_get_optimization_job_status_found(self, mock_optimization_manager):
        """
        When the job exists, /optimization/{job_id}/status must return the
        OptimizationJobStatus with HTTP 200.
        """
        graph = self._make_minimal_graph()
        mock_optimization_manager.get_job_status.return_value = (
            schemas.OptimizationJobStatus(
                id="job-status-1",
                type=schemas.OptimizationType.OPTIMIZE,
                start_time=123456,
                elapsed_time=1000,
                state=schemas.OptimizationJobState.RUNNING,
                total_fps=None,
                original_pipeline_graph=graph,
                optimized_pipeline_graph=None,
                original_pipeline_description="filesrc ! decodebin ! sink",
                optimized_pipeline_description=None,
                error_message=None,
            )
        )

        response = self.client.get("/jobs/optimization/job-status-1/status")

        self.assertEqual(response.status_code, 200)
        data = response.json()

        self.assertEqual(data["id"], "job-status-1")
        self.assertEqual(data["type"], schemas.OptimizationType.OPTIMIZE)
        self.assertEqual(data["state"], schemas.OptimizationJobState.RUNNING)
        self.assertIn("original_pipeline_graph", data)

        mock_optimization_manager.get_job_status.assert_called_once_with("job-status-1")

    @patch("api.routes.jobs.optimization_manager")
    def test_get_optimization_job_status_not_found(self, mock_optimization_manager):
        """
        When the job does not exist, /optimization/{job_id}/status must
        respond with HTTP 404 and a MessageResponse.
        """
        mock_optimization_manager.get_job_status.return_value = None

        missing_job_id = "unknown-status-job"
        response = self.client.get(f"/jobs/optimization/{missing_job_id}/status")

        self.assertEqual(response.status_code, 404)
        self.assertEqual(
            response.json(),
            schemas.MessageResponse(
                message=f"Optimization job {missing_job_id} not found"
            ).model_dump(),
        )

        mock_optimization_manager.get_job_status.assert_called_once_with(missing_job_id)

    # ------------------------------------------------------------------
    # Router metadata
    # ------------------------------------------------------------------

    def test_operation_ids_are_exposed_as_expected(self):
        """
        Ensure that the router in jobs.py is configured with the expected
        ``operation_id`` values.

        This test does not perform any HTTP calls; instead it inspects
        the FastAPI route definitions.  This is useful to:

        * keep the OpenAPI schema stable,
        * catch accidental renames of operation IDs,
        * slightly increase coverage on routing-related code paths.
        """
        # Collect mapping from path+method to operation_id
        operations = {}
        for route in jobs_router.routes:
            if not isinstance(route, APIRoute):
                # Skip non-HTTP routes such as WebSocketRoute.
                continue
            for method in route.methods:
                operations[(route.path, method)] = route.operation_id

        self.assertIn(("/optimization/status", "GET"), operations)
        self.assertIn(("/optimization/{job_id}", "GET"), operations)
        self.assertIn(("/optimization/{job_id}/status", "GET"), operations)

        self.assertEqual(
            operations[("/optimization/status", "GET")],
            "get_optimization_statuses",
        )
        self.assertEqual(
            operations[("/optimization/{job_id}", "GET")],
            "get_optimization_job_summary",
        )
        self.assertEqual(
            operations[("/optimization/{job_id}/status", "GET")],
            "get_optimization_job_status",
        )


if __name__ == "__main__":
    # Allow running this module directly for quick local verification.
    unittest.main()

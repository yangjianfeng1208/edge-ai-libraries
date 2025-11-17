import time
import unittest
from unittest.mock import patch

from api.api_schemas import (
    PipelineRequestRun,
    PipelineRunSpec,
    PipelineBenchmarkSpec,
    PipelineRequestBenchmark,
    PipelineInstanceState,
    PipelineType,
)
from managers.instance_manager import InstanceManager, PipelineInstance


class TestInstanceManager(unittest.TestCase):
    test_graph = """
    {
        "nodes": [
            {
                "id": "0",
                "type": "filesrc",
                "data": {
                    "location": "/tmp/dummy-video.mp4"
                }
            },
            {
                "id": "1",
                "type": "decodebin3",
                "data": {}
            },
            {
                "id": "2",
                "type": "autovideosink",
                "data": {}
            }
        ],
        "edges": [
            {
                "id": "0",
                "source": "0",
                "target": "1"
            },
            {
                "id": "1",
                "source": "1",
                "target": "2"
            }
        ]
    }
    """

    def test_run_pipeline_calls_execute_pipeline_and_returns_instance_id(self):
        manager = InstanceManager()
        initial_count = len(manager.instances)

        pipeline_request = PipelineRequestRun(
            pipeline_run_specs=[
                PipelineRunSpec(
                    name="user-defined-pipelines",
                    version="test-pipeline",
                    streams=1,
                )
            ]
        )

        with patch.object(manager, "_execute_pipeline") as mock_execute:
            instance_id = manager.run_pipeline(
                "user-defined-pipelines", "test-pipeline", pipeline_request
            )
            self.assertIsInstance(instance_id, str)
            self.assertIn(instance_id, manager.instances)
            self.assertEqual(initial_count + 1, len(manager.instances))
            mock_execute.assert_called_once_with(
                instance_id, "user-defined-pipelines", "test-pipeline", pipeline_request
            )

    def test_run_pipeline_creates_instance_with_running_state(self):
        manager = InstanceManager()
        pipeline_request = PipelineRequestRun(
            pipeline_run_specs=[
                PipelineRunSpec(
                    name="user-defined-pipelines",
                    version="test-pipeline",
                    streams=1,
                )
            ]
        )

        with patch.object(manager, "_execute_pipeline"):
            instance_id = manager.run_pipeline(
                "user-defined-pipelines", "test-pipeline", pipeline_request
            )
            instance = manager.instances[instance_id]
            self.assertEqual(instance.name, "user-defined-pipelines")
            self.assertEqual(instance.version, "test-pipeline")
            self.assertEqual(instance.request, pipeline_request)
            self.assertEqual(instance.state.name, PipelineInstanceState.RUNNING)
            self.assertIsInstance(instance.start_time, int)
            self.assertIsNone(instance.end_time)

    def test_benchmark_pipeline_creates_instance_with_running_state_and_returns_instance_id(
        self,
    ):
        manager = InstanceManager()
        initial_count = len(manager.instances)

        pipeline_request = PipelineRequestBenchmark(
            fps_floor=30,
            pipeline_specs=[
                PipelineBenchmarkSpec(
                    name="user-defined-pipelines",
                    version="test-pipeline",
                    stream_rate=100,
                )
            ],
        )

        with patch.object(manager, "_execute_benchmark") as mock_execute:
            instance_id = manager.benchmark_pipeline(
                "user-defined-pipelines", "test-pipeline", pipeline_request
            )
            self.assertIsInstance(instance_id, str)
            self.assertEqual(initial_count + 1, len(manager.instances))

            instance = manager.instances[instance_id]
            self.assertEqual(instance.name, "user-defined-pipelines")
            self.assertEqual(instance.version, "test-pipeline")
            self.assertEqual(instance.request, pipeline_request)
            self.assertEqual(instance.state.name, PipelineInstanceState.RUNNING)
            self.assertIsInstance(instance.start_time, int)
            self.assertIsNone(instance.end_time)

            mock_execute.assert_called_once_with(
                instance_id, "user-defined-pipelines", "test-pipeline", pipeline_request
            )

    def test_get_all_instance_statuses_returns_correct_statuses(self):
        manager = InstanceManager()

        # Create two instances
        pipeline_request_run = PipelineRequestRun(
            pipeline_run_specs=[
                PipelineRunSpec(
                    name="user-defined-pipelines",
                    version="test-pipeline",
                    streams=1,
                )
            ]
        )

        pipeline_request_benchmark = PipelineRequestBenchmark(
            fps_floor=30,
            pipeline_specs=[
                PipelineBenchmarkSpec(
                    name="user-defined-pipelines",
                    version="test-pipeline",
                    stream_rate=100,
                )
            ],
        )

        with (
            patch.object(manager, "_execute_pipeline"),
            patch.object(manager, "_execute_benchmark"),
        ):
            instance_id_run = manager.run_pipeline(
                "user-defined-pipelines", "test-pipeline", pipeline_request_run
            )
            instance_id_benchmark = manager.benchmark_pipeline(
                "user-defined-pipelines", "test-pipeline", pipeline_request_benchmark
            )

        statuses = manager.get_all_instance_statuses()
        self.assertEqual(len(statuses), 2)

        status_run = next((s for s in statuses if s.id == instance_id_run), None)
        status_benchmark = next(
            (s for s in statuses if s.id == instance_id_benchmark), None
        )

        self.assertIsNotNone(status_run)
        self.assertIsNotNone(status_benchmark)
        assert status_run is not None  # for pyright type checking
        assert status_benchmark is not None  # for pyright type checking
        self.assertEqual(status_run.state.name, PipelineInstanceState.RUNNING)
        self.assertEqual(status_benchmark.state.name, PipelineInstanceState.RUNNING)

    def test_get_instance_status_returns_none_for_nonexistent_instance(self):
        manager = InstanceManager()
        status = manager.get_instance_status("nonexistent-instance-id")
        self.assertIsNone(status)

    def test_get_instance_status_returns_correct_status(self):
        manager = InstanceManager()

        # Create an instance manually and add it to the manager
        pipeline_request_run = PipelineRequestRun(
            pipeline_run_specs=[
                PipelineRunSpec(
                    name="user-defined-pipelines",
                    version="test-pipeline",
                    streams=1,
                )
            ]
        )
        instance = PipelineInstance(
            id="test-instance-id",
            name="user-defined-pipelines",
            version="test-pipeline",
            request=pipeline_request_run,
            start_time=int(time.time()),
            state=PipelineInstanceState.RUNNING,
            total_fps=120,
            per_stream_fps=30,
            total_streams=1,
            streams_per_pipeline=pipeline_request_run.pipeline_run_specs,
        )
        manager.instances[instance.id] = instance

        status = manager.get_instance_status(instance.id)
        self.assertIsNotNone(status)
        assert status is not None  # for pyright type checking
        self.assertEqual(status.id, instance.id)
        self.assertEqual(status.state, instance.state)
        self.assertEqual(status.total_fps, instance.total_fps)
        self.assertEqual(status.per_stream_fps, instance.per_stream_fps)
        self.assertEqual(status.total_streams, instance.total_streams)
        self.assertEqual(status.streams_per_pipeline, instance.streams_per_pipeline)

    def test_get_instance_summary_returns_none_for_nonexistent_instance(self):
        manager = InstanceManager()
        summary = manager.get_instance_summary("nonexistent-instance-id")
        self.assertIsNone(summary)

    def test_get_instance_summary_returns_correct_summary(self):
        manager = InstanceManager()

        # Create an instance manually and add it to the manager
        pipeline_request = PipelineRequestRun(
            pipeline_run_specs=[
                PipelineRunSpec(
                    name="user-defined-pipelines",
                    version="test-pipeline",
                    streams=1,
                )
            ]
        )

        instance = PipelineInstance(
            id="test-instance-id",
            name="user-defined-pipelines",
            version="test-pipeline",
            request=pipeline_request,
            start_time=int(time.time()),
            state=PipelineInstanceState.RUNNING,
        )
        manager.instances[instance.id] = instance

        summary = manager.get_instance_summary(instance.id)
        self.assertIsNotNone(summary)
        assert summary is not None  # for pyright type checking
        self.assertEqual(summary.id, instance.id)
        self.assertEqual(summary.request, instance.request)
        self.assertEqual(summary.type, PipelineType.GSTREAMER)

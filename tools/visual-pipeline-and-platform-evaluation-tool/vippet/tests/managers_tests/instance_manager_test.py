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
from pipeline_runner import PipelineRunner
from benchmark import BenchmarkResult
from managers.instance_manager import InstanceManager, PipelineInstance


class TestInstanceManager(unittest.TestCase):
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
            pipeline_benchmark_specs=[
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
            pipeline_benchmark_specs=[
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

    def test_stop_instance_stops_running_instance(self):
        manager = InstanceManager()

        # Create an instance and runner manually and add them to the manager
        pipeline_request = PipelineRequestRun(
            pipeline_run_specs=[
                PipelineRunSpec(
                    name="user-defined-pipelines",
                    version="test-pipeline",
                    streams=1,
                )
            ]
        )

        instance_id = "test-instance-id"
        instance = PipelineInstance(
            id=instance_id,
            name="user-defined-pipelines",
            version="test-pipeline",
            request=pipeline_request,
            start_time=int(time.time()),
            state=PipelineInstanceState.RUNNING,
        )
        manager.instances[instance_id] = instance
        manager.runners[instance_id] = PipelineRunner()

        success, message = manager.stop_instance(instance_id)
        self.assertTrue(success)
        self.assertIn(f"Instance {instance_id} stopped", message)

    def test_stop_instance_returns_false_for_nonexistent_instance(self):
        manager = InstanceManager()
        success, message = manager.stop_instance("nonexistent-instance-id")
        self.assertFalse(success)
        self.assertIn("not found", message)

    def test_stop_instance_returns_false_for_nonexistent_runner(self):
        manager = InstanceManager()

        # Create an instance without a runner
        pipeline_request = PipelineRequestRun(
            pipeline_run_specs=[
                PipelineRunSpec(
                    name="user-defined-pipelines",
                    version="test-pipeline",
                    streams=1,
                )
            ]
        )

        instance_id = "test-instance-id"
        instance = PipelineInstance(
            id=instance_id,
            name="user-defined-pipelines",
            version="test-pipeline",
            request=pipeline_request,
            start_time=int(time.time()),
            state=PipelineInstanceState.RUNNING,
        )
        manager.instances[instance_id] = instance

        success, message = manager.stop_instance(instance_id)
        self.assertFalse(success)
        self.assertIn(
            f"No active runner found for instance {instance_id}. It may have already completed or was never started.",
            message,
        )

    def test_stop_instance_returns_false_for_not_running_instance(self):
        manager = InstanceManager()

        # Create an instance that is not running
        pipeline_request = PipelineRequestRun(
            pipeline_run_specs=[
                PipelineRunSpec(
                    name="user-defined-pipelines",
                    version="test-pipeline",
                    streams=1,
                )
            ]
        )

        instance_id = "test-instance-id"
        instance = PipelineInstance(
            id=instance_id,
            name="user-defined-pipelines",
            version="test-pipeline",
            request=pipeline_request,
            start_time=int(time.time()),
            state=PipelineInstanceState.COMPLETED,
        )
        manager.instances[instance_id] = instance
        manager.runners[instance_id] = PipelineRunner()

        success, message = manager.stop_instance(instance_id)
        self.assertFalse(success)
        self.assertIn(f"Instance {instance_id} is not running", message)

    def test_execute_pipeline_starts_pipeline(self):
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

        instance_id = "test-instance-start"
        instance = PipelineInstance(
            id=instance_id,
            name="user-defined-pipelines",
            version="test-pipeline",
            request=pipeline_request,
            start_time=int(time.time()),
            state=PipelineInstanceState.RUNNING,
        )
        manager.instances[instance_id] = instance

        with (
            patch(
                "managers.instance_manager.pipeline_manager.build_pipeline_command",
                return_value="fakesrc ! fakesink",
            ),
            patch.object(PipelineRunner, "run", return_value=None) as mock_run,
        ):
            manager._execute_pipeline(
                instance_id,
                "user-defined-pipelines",
                "test-pipeline",
                pipeline_request,
            )
            self.assertIn(instance_id, manager.instances)
            mock_run.assert_called_once()

    def test_execute_pipeline_updates_metrics_on_completion(self):
        from pipeline_runner import PipelineRunResult

        manager = InstanceManager()

        pipeline_request = PipelineRequestRun(
            pipeline_run_specs=[
                PipelineRunSpec(name="pipeA", version="v1", streams=1),
                PipelineRunSpec(name="pipeB", version="v2", streams=2),
            ]
        )

        instance_id = "test-instance-metrics"
        instance = PipelineInstance(
            id=instance_id,
            name="user-defined-pipelines",
            version="test-pipeline",
            request=pipeline_request,
            start_time=int(time.time()),
            state=PipelineInstanceState.RUNNING,
        )
        manager.instances[instance_id] = instance

        with (
            patch(
                "managers.instance_manager.pipeline_manager.build_pipeline_command",
                return_value="fakesrc ! fakesink",
            ),
            patch.object(
                PipelineRunner,
                "run",
                return_value=PipelineRunResult(
                    total_fps=300.0, per_stream_fps=100.0, num_streams=3
                ),
            ),
            patch.object(PipelineRunner, "is_cancelled", return_value=False),
        ):
            manager._execute_pipeline(
                instance_id,
                "user-defined-pipelines",
                "test-pipeline",
                pipeline_request,
            )

        updated = manager.instances[instance_id]
        self.assertEqual(updated.state, PipelineInstanceState.COMPLETED)
        self.assertEqual(updated.total_fps, 300.0)
        self.assertEqual(updated.per_stream_fps, 100.0)
        self.assertEqual(updated.total_streams, 3)
        self.assertIsNotNone(updated.streams_per_pipeline)
        self.assertEqual(len(updated.streams_per_pipeline or []), 2)
        self.assertNotIn(instance_id, manager.runners)

    def test_execute_pipeline_aborts_on_cancelled_runner(self):
        from pipeline_runner import PipelineRunResult

        manager = InstanceManager()
        pipeline_request = PipelineRequestRun(
            pipeline_run_specs=[
                PipelineRunSpec(name="pipeA", version="v1", streams=1),
            ]
        )

        instance_id = "test-instance-cancel"
        instance = PipelineInstance(
            id=instance_id,
            name="user-defined-pipelines",
            version="test-pipeline",
            request=pipeline_request,
            start_time=int(time.time()),
            state=PipelineInstanceState.RUNNING,
        )
        manager.instances[instance_id] = instance

        with (
            patch(
                "managers.instance_manager.pipeline_manager.build_pipeline_command",
                return_value="fakesrc ! fakesink",
            ),
            patch.object(
                PipelineRunner,
                "run",
                return_value=PipelineRunResult(
                    total_fps=100.0, per_stream_fps=100.0, num_streams=1
                ),
            ),
            patch.object(PipelineRunner, "is_cancelled", return_value=True),
        ):
            manager._execute_pipeline(
                instance_id,
                "user-defined-pipelines",
                "test-pipeline",
                pipeline_request,
            )

        updated = manager.instances[instance_id]
        self.assertEqual(updated.state, PipelineInstanceState.ABORTED)
        self.assertEqual(updated.error_message, "Cancelled by user")
        self.assertNotIn(instance_id, manager.runners)

    def test_execute_pipeline_sets_error_on_exception(self):
        manager = InstanceManager()
        pipeline_request = PipelineRequestRun(
            pipeline_run_specs=[
                PipelineRunSpec(name="pipeA", version="v1", streams=1),
            ]
        )

        instance_id = "test-instance-exception"
        instance = PipelineInstance(
            id=instance_id,
            name="user-defined-pipelines",
            version="test-pipeline",
            request=pipeline_request,
            start_time=int(time.time()),
            state=PipelineInstanceState.RUNNING,
        )
        manager.instances[instance_id] = instance

        with patch(
            "managers.instance_manager.pipeline_manager.build_pipeline_command",
            side_effect=ValueError("boom"),
        ):
            manager._execute_pipeline(
                instance_id,
                "user-defined-pipelines",
                "test-pipeline",
                pipeline_request,
            )

        updated = manager.instances[instance_id]
        self.assertEqual(updated.state, PipelineInstanceState.ERROR)
        self.assertIn("boom", updated.error_message or "")
        self.assertNotIn(instance_id, manager.runners)

    @patch("managers.instance_manager.Benchmark.run")
    def test_execute_benchmark_updates_metrics_on_completion(self, mock_benchmark_run):
        mock_benchmark_run.return_value = BenchmarkResult(
            n_streams=3,
            streams_per_pipeline=[
                PipelineRunSpec(name="pipeA", version="v1", streams=2),
                PipelineRunSpec(name="pipeB", version="v2", streams=1),
            ],
            per_stream_fps=90.0,
        )

        manager = InstanceManager()
        pipeline_request = PipelineRequestBenchmark(
            fps_floor=30,
            pipeline_benchmark_specs=[
                PipelineBenchmarkSpec(name="pipeA", version="v1", stream_rate=50),
                PipelineBenchmarkSpec(name="pipeB", version="v2", stream_rate=50),
            ],
        )

        instance_id = "test-benchmark-success"
        instance = PipelineInstance(
            id=instance_id,
            name="user-defined-pipelines",
            version="test-benchmark",
            request=pipeline_request,
            start_time=int(time.time()),
            state=PipelineInstanceState.RUNNING,
        )
        manager.instances[instance_id] = instance

        mock_benchmark_run.return_value = BenchmarkResult(
            n_streams=3,
            streams_per_pipeline=[
                PipelineRunSpec(name="pipeA", version="v1", streams=2),
                PipelineRunSpec(name="pipeB", version="v2", streams=1),
            ],
            per_stream_fps=90.0,
        )

        manager._execute_benchmark(
            instance_id, "user-defined-pipelines", "test-benchmark", pipeline_request
        )

        updated = manager.instances[instance_id]
        self.assertEqual(updated.state, PipelineInstanceState.COMPLETED)
        self.assertIsNone(updated.total_fps)  # benchmark does not set total_fps
        self.assertEqual(updated.per_stream_fps, 90.0)
        self.assertEqual(len(updated.streams_per_pipeline or []), 2)
        self.assertEqual(updated.total_streams, 3)
        self.assertNotIn(instance_id, manager.runners)

    def test_execute_benchmark_aborts_on_cancelled_runner(self):
        manager = InstanceManager()
        benchmark_request = PipelineRequestBenchmark(
            fps_floor=30,
            pipeline_benchmark_specs=[
                PipelineBenchmarkSpec(name="pipeA", version="v1", stream_rate=100),
            ],
        )

        instance_id = "test-benchmark-cancel"
        instance = PipelineInstance(
            id=instance_id,
            name="user-defined-pipelines",
            version="test-benchmark",
            request=benchmark_request,
            start_time=int(time.time()),
            state=PipelineInstanceState.RUNNING,
        )
        manager.instances[instance_id] = instance

        # Patch the instance's runner.is_cancelled to return True
        with patch("managers.instance_manager.Benchmark") as MockBenchmark:
            mock_benchmark_instance = MockBenchmark.return_value
            mock_benchmark_instance.run.return_value = BenchmarkResult(
                n_streams=3,
                streams_per_pipeline=[
                    PipelineRunSpec(name="pipeA", version="v1", streams=2),
                    PipelineRunSpec(name="pipeB", version="v2", streams=1),
                ],
                per_stream_fps=90.0,
            )
            mock_benchmark_instance.runner.is_cancelled.return_value = True

            manager._execute_benchmark(
                instance_id,
                "user-defined-pipelines",
                "test-benchmark",
                benchmark_request,
            )

        updated = manager.instances[instance_id]
        self.assertEqual(updated.state, PipelineInstanceState.ABORTED)
        self.assertEqual(updated.error_message, "Cancelled by user")
        self.assertIsNone(updated.per_stream_fps)
        self.assertIsNone(updated.streams_per_pipeline)
        self.assertNotIn(instance_id, manager.runners)

    def test_execute_benchmark_sets_error_on_exception(self):
        manager = InstanceManager()
        benchmark_request = PipelineRequestBenchmark(
            fps_floor=30,
            pipeline_benchmark_specs=[
                PipelineBenchmarkSpec(name="pipeA", version="v1", stream_rate=100),
            ],
        )

        instance_id = "test-benchmark-exception"
        instance = PipelineInstance(
            id=instance_id,
            name="user-defined-pipelines",
            version="test-benchmark",
            request=benchmark_request,
            start_time=int(time.time()),
            state=PipelineInstanceState.RUNNING,
        )
        manager.instances[instance_id] = instance

        with patch("managers.instance_manager.Benchmark") as MockBenchmark:
            mock_benchmark_instance = MockBenchmark.return_value
            # Use side_effect to make the mocked run() raise an exception
            mock_benchmark_instance.run.side_effect = RuntimeError("benchmark failed")

            manager._execute_benchmark(
                instance_id,
                "user-defined-pipelines",
                "test-benchmark",
                benchmark_request,
            )

        updated = manager.instances[instance_id]
        self.assertEqual(updated.state, PipelineInstanceState.ERROR)
        self.assertIn("benchmark failed", updated.error_message or "")
        self.assertNotIn(instance_id, manager.runners)

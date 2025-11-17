import unittest
from unittest.mock import patch

from benchmark import Benchmark, BenchmarkResult, PipelineBenchmarkSpec, PipelineRunSpec
from pipeline_runner import PipelineRunResult


class TestBenchmark(unittest.TestCase):
    def setUp(self):
        self.fps_floor = 30
        self.pipeline_specs = [
            PipelineBenchmarkSpec(
                name="test-pipeline-1", version="1.0", stream_rate=50
            ),
            PipelineBenchmarkSpec(
                name="test-pipeline-2", version="1.0", stream_rate=50
            ),
        ]
        self.benchmark = Benchmark()

    @patch("benchmark.pipeline_manager.build_pipeline_command")
    def test_run_successful_scaling(self, mock_build_command):
        mock_build_command.return_value = ""  # No actual command needed for test
        expected_result = BenchmarkResult(
            n_streams=3,
            streams_per_pipeline=[
                PipelineRunSpec(
                    name="test-pipeline-1",
                    version="1.0",
                    streams=2,
                ),
                PipelineRunSpec(
                    name="test-pipeline-2",
                    version="1.0",
                    streams=1,
                ),
            ],
            per_stream_fps=31.0,
        )

        with patch.object(self.benchmark.runner, "run") as mock_runner:
            mock_runner.side_effect = [
                # First call with 1 stream
                PipelineRunResult(
                    total_fps=30,
                    per_stream_fps=30,
                    num_streams=1,
                ),
                # Second call with 2 streams
                PipelineRunResult(
                    total_fps=80,
                    per_stream_fps=40,
                    num_streams=2,
                ),
                # Third call with 4 streams
                PipelineRunResult(
                    total_fps=100,
                    per_stream_fps=25,
                    num_streams=4,
                ),
                # Fourth call with 3 streams
                PipelineRunResult(
                    total_fps=93,
                    per_stream_fps=31,
                    num_streams=3,
                ),
                # Fourth call with 3 streams
                PipelineRunResult(
                    total_fps=93,
                    per_stream_fps=31,
                    num_streams=3,
                ),
                # Fifth call with 4 streams
                PipelineRunResult(
                    total_fps=100,
                    per_stream_fps=25,
                    num_streams=4,
                ),
            ]

            result = self.benchmark.run(self.pipeline_specs, fps_floor=self.fps_floor)

            self.assertEqual(result, expected_result)

    @patch("benchmark.pipeline_manager.build_pipeline_command")
    def test_zero_total_fps(self, mock_build_command):
        mock_build_command.return_value = ""  # No actual command needed for test
        with patch.object(self.benchmark.runner, "run") as mock_runner:
            mock_runner.side_effect = [
                # First call with 1 stream
                PipelineRunResult(total_fps=0, per_stream_fps=30, num_streams=1),
            ]
            with self.assertRaises(
                RuntimeError, msg="Pipeline returned zero or invalid FPS metrics."
            ):
                _ = self.benchmark.run(self.pipeline_specs, fps_floor=self.fps_floor)

    @patch("benchmark.pipeline_manager.build_pipeline_command")
    def test_pipeline_returns_none(self, mock_build_command):
        mock_build_command.return_value = ""  # No actual command needed for test
        with patch.object(self.benchmark.runner, "run") as mock_runner:
            mock_runner.side_effect = [None]

            with self.assertRaises(
                RuntimeError, msg="Pipeline runner returned invalid results."
            ):
                _ = self.benchmark.run(self.pipeline_specs, fps_floor=self.fps_floor)


if __name__ == "__main__":
    unittest.main()

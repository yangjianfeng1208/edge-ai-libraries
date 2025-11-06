"""benchmark.py

This module provides the Benchmark class for evaluating pipeline performance
based on configurable parameters and stream counts.
"""

import logging
from dataclasses import dataclass
import math

from pipeline_runner import PipelineRunner, PipelineRunResult
from gstpipeline import GstPipeline


@dataclass
class BenchmarkResult:
    n_streams: int
    ai_streams: int
    non_ai_streams: int
    per_stream_fps: float

    def __repr__(self):
        return (
            f"BenchmarkResult("
            f"n_streams={self.n_streams}, "
            f"ai_streams={self.ai_streams}, "
            f"non_ai_streams={self.non_ai_streams}, "
            f"per_stream_fps={self.per_stream_fps}"
            f")"
        )


class Benchmark:
    """Benchmarking class for pipeline evaluation."""

    def __init__(self):
        self.best_result = None
        self.runner = PipelineRunner()
        self.logger = logging.getLogger(__name__)

    def run(
        self, pipeline_description: GstPipeline, fps_floor: float, rate: int
    ) -> BenchmarkResult:
        """Run the benchmark and return the best configuration."""
        n_streams = 1
        per_stream_fps = 0.0
        exponential = True
        lower_bound = 1
        # We'll set this once we fall below the fps_floor
        higher_bound = -1
        best_config = (0, 0, 0, 0.0)

        while True:
            ai_streams = math.ceil(n_streams * (rate / 100))
            non_ai_streams = n_streams - ai_streams

            results = self.runner.run(pipeline_description, non_ai_streams, ai_streams)

            # Check for cancellation
            if self.runner.is_cancelled():
                self.logger.info("Benchmark cancelled.")
                break

            if results is None or not isinstance(results, PipelineRunResult):
                raise RuntimeError("Pipeline runner returned invalid results.")

            try:
                total_fps = results.total_fps
                per_stream_fps = total_fps / n_streams if n_streams > 0 else 0.0
            except (ValueError, TypeError, ZeroDivisionError):
                raise RuntimeError("Failed to parse FPS metrics from pipeline results.")
            if total_fps == 0 or math.isnan(per_stream_fps):
                raise RuntimeError("Pipeline returned zero or invalid FPS metrics.")

            self.logger.info(
                "n_streams=%d, total_fps=%f, per_stream_fps=%f, exponential=%s, lower_bound=%d, higher_bound=%s",
                n_streams,
                total_fps,
                per_stream_fps,
                exponential,
                lower_bound,
                higher_bound,
            )

            # increase number of streams exponentially until we drop below fps_floor
            if exponential:
                if per_stream_fps >= fps_floor:
                    best_config = (
                        n_streams,
                        ai_streams,
                        non_ai_streams,
                        per_stream_fps,
                    )
                    n_streams *= 2
                else:
                    exponential = False
                    higher_bound = n_streams
                    lower_bound = n_streams // 2
                    n_streams = (lower_bound + higher_bound) // 2
            # use bisecting search for fine tune maximum number of streams
            else:
                if per_stream_fps >= fps_floor:
                    best_config = (
                        n_streams,
                        ai_streams,
                        non_ai_streams,
                        per_stream_fps,
                    )
                    lower_bound = n_streams + 1
                else:
                    higher_bound = n_streams - 1

                if lower_bound > higher_bound:
                    break  # Binary search complete

                n_streams = (lower_bound + higher_bound) // 2

            if n_streams <= 0:
                n_streams = 1  # Prevent N from going below 1

        if best_config[0] > 0:
            bm_result = BenchmarkResult(
                n_streams=best_config[0],
                ai_streams=best_config[1],
                non_ai_streams=best_config[2],
                per_stream_fps=best_config[3],
            )
        else:
            bm_result = BenchmarkResult(
                n_streams=n_streams,
                ai_streams=ai_streams,
                non_ai_streams=non_ai_streams,
                per_stream_fps=per_stream_fps,
            )

        return bm_result

    def cancel(self):
        """Cancel the ongoing benchmark."""
        self.runner.cancel()

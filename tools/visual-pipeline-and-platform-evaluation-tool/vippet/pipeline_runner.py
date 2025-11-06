"""pipeline_runner.py

This module provides the PipelineRunner class for executing GStreamer pipelines
and extracting performance metrics (FPS).
"""

import logging
import os
import re
import select
import psutil as ps
from dataclasses import dataclass
from subprocess import PIPE, Popen

from gstpipeline import GstPipeline


@dataclass
class PipelineRunResult:
    """Result of a pipeline run with FPS metrics."""

    total_fps: float
    per_stream_fps: float
    num_streams: int

    def __repr__(self):
        return (
            f"PipelineRunResult("
            f"total_fps={self.total_fps}, "
            f"per_stream_fps={self.per_stream_fps}, "
            f"num_streams={self.num_streams}"
            f")"
        )


class PipelineRunner:
    """
    A class for running GStreamer pipelines and extracting FPS metrics.

    This class handles the execution of GStreamer pipeline commands and
    parses the output to extract performance metrics such as total FPS
    and per-stream FPS.
    """

    # Default path to the FPS file
    DEFAULT_FPS_FILE_PATH = "/home/dlstreamer/vippet/.collector-signals/fps.txt"

    def __init__(self, poll_interval: int = 1, fps_file_path: str | None = None):
        """
        Initialize the PipelineRunner.

        Args:
            poll_interval: Interval in seconds to poll the process for metrics.
            fps_file_path: Optional path to write latest FPS values (for real-time monitoring).
        """
        self.poll_interval = poll_interval
        self.fps_file_path = fps_file_path or self.DEFAULT_FPS_FILE_PATH
        self.logger = logging.getLogger("PipelineRunner")
        self.cancelled = False

    def run(
        self,
        pipeline_description: GstPipeline,
        regular_channels: int = 0,
        inference_channels: int = 1,
    ) -> PipelineRunResult:
        """
        Run a GStreamer pipeline and extract FPS metrics.

        Args:
            pipeline_description (GstPipeline): The GStreamer pipeline description to execute.
            regular_channels: Number of regular (non-AI) channels.
            inference_channels: Number of inference (AI) channels.

        Returns:
            PipelineRunResult containing total_fps, per_stream_fps, and num_streams.

        Raises:
            RuntimeError: If pipeline execution fails.
        """
        # Construct the pipeline command
        pipeline_cmd = "gst-launch-1.0 -q " + pipeline_description.evaluate(
            regular_channels, inference_channels
        )

        total_channels = inference_channels + regular_channels

        # Log the command
        self.logger.info(f"Pipeline Command: {pipeline_cmd}")

        try:
            # Set the environment variable to enable all drivers
            env = os.environ.copy()
            env["GST_VA_ALL_DRIVERS"] = "1"

            # Spawn command in a subprocess
            process = Popen(pipeline_cmd.split(" "), stdout=PIPE, stderr=PIPE, env=env)

            exit_code = None
            total_fps = None
            per_stream_fps = None
            num_streams = None
            last_fps = None
            avg_fps_dict = {}
            process_output = []
            process_stderr = []

            # Define pattern to capture FPSCounter metrics
            overall_pattern = r"FpsCounter\(overall ([\d.]+)sec\): total=([\d.]+) fps, number-streams=(\d+), per-stream=([\d.]+) fps"
            avg_pattern = r"FpsCounter\(average ([\d.]+)sec\): total=([\d.]+) fps, number-streams=(\d+), per-stream=([\d.]+) fps"
            last_pattern = r"FpsCounter\(last ([\d.]+)sec\): total=([\d.]+) fps, number-streams=(\d+), per-stream=([\d.]+) fps"

            # Poll the process to check if it is still running
            while process.poll() is None:
                if self.cancelled:
                    process.terminate()
                    self.logger.info("Process cancelled, terminating")
                    break

                reads, _, _ = select.select(
                    [process.stdout, process.stderr], [], [], self.poll_interval
                )
                for r in reads:
                    line = r.readline()
                    if not line:
                        continue

                    if r == process.stdout:
                        process_output.append(line)

                        # Write the average FPS to file in real-time for monitoring
                        line_str = line.decode("utf-8")
                        match = re.search(avg_pattern, line_str)
                        if match:
                            result = {
                                "total_fps": float(match.group(2)),
                                "number_streams": int(match.group(3)),
                                "per_stream_fps": float(match.group(4)),
                            }
                            self.logger.info(
                                f"Avg FPS: {result['total_fps']} fps; "
                                f"Num Streams: {result['number_streams']}; "
                                f"Per Stream FPS: {result['per_stream_fps']} fps."
                            )

                            # Skip the result if the number of streams does not match
                            if result["number_streams"] != total_channels:
                                continue

                            latest_fps = result["per_stream_fps"]

                            # Write latest FPS to file
                            try:
                                with open(self.fps_file_path, "w") as f:
                                    f.write(f"{latest_fps}\n")
                            except (OSError, IOError) as e:
                                self.logger.warning(f"Failed to write FPS to file: {e}")

                    elif r == process.stderr:
                        process_stderr.append(line)

                    try:
                        if ps.Process(process.pid).status() == "zombie":
                            exit_code = process.wait()
                            break
                    except ps.NoSuchProcess:
                        # Process has already terminated
                        exit_code = process.wait()
                        break

            # Capture any remaining output after process ends
            if exit_code is None:
                exit_code = process.wait()

            # Process the output and extract FPS metrics
            for line in process_output:
                line_str = line.decode("utf-8")

                match = re.search(overall_pattern, line_str)
                if match:
                    result = {
                        "total_fps": float(match.group(2)),
                        "number_streams": int(match.group(3)),
                        "per_stream_fps": float(match.group(4)),
                    }
                    if result["number_streams"] == total_channels:
                        total_fps = result["total_fps"]
                        num_streams = result["number_streams"]
                        per_stream_fps = result["per_stream_fps"]
                        break

                match = re.search(avg_pattern, line_str)
                if match:
                    result = {
                        "total_fps": float(match.group(2)),
                        "number_streams": int(match.group(3)),
                        "per_stream_fps": float(match.group(4)),
                    }
                    avg_fps_dict[result["number_streams"]] = result

                match = re.search(last_pattern, line_str)
                if match:
                    result = {
                        "total_fps": float(match.group(2)),
                        "number_streams": int(match.group(3)),
                        "per_stream_fps": float(match.group(4)),
                    }
                    last_fps = result

            # Fallback to average FPS if overall not found
            if total_fps is None and avg_fps_dict.keys():
                if total_channels in avg_fps_dict.keys():
                    total_fps = avg_fps_dict[total_channels]["total_fps"]
                    num_streams = avg_fps_dict[total_channels]["number_streams"]
                    per_stream_fps = avg_fps_dict[total_channels]["per_stream_fps"]
                else:
                    # Find closest match
                    closest_match = min(
                        avg_fps_dict.keys(),
                        key=lambda x: abs(x - total_channels),
                        default=None,
                    )
                    if closest_match is not None:
                        total_fps = avg_fps_dict[closest_match]["total_fps"]
                        num_streams = avg_fps_dict[closest_match]["number_streams"]
                        per_stream_fps = avg_fps_dict[closest_match]["per_stream_fps"]

            # Fallback to last FPS if average not found
            if total_fps is None and last_fps:
                total_fps = last_fps["total_fps"]
                num_streams = last_fps["number_streams"]
                per_stream_fps = last_fps["per_stream_fps"]

            # Convert None to appropriate values
            if total_fps is None:
                total_fps = 0.0
            if num_streams is None:
                num_streams = 0
            if per_stream_fps is None:
                per_stream_fps = 0.0

            # Save results
            stdout_str = "".join(
                [line.decode("utf-8", errors="replace") for line in process_output]
            )
            stderr_str = "".join(
                [line.decode("utf-8", errors="replace") for line in process_stderr]
            )

            # Log errors if exit_code is not zero
            if exit_code != 0:
                self.logger.error("Pipeline failed with exit_code=%s", exit_code)
                self.logger.error("STDOUT:\n%s", stdout_str)
                self.logger.error("STDERR:\n%s", stderr_str)
                # Only raise an error if the failure was not due to cancellation
                if not self.is_cancelled():
                    raise RuntimeError(
                        f"Pipeline execution failed: {stderr_str.strip()}"
                    )

            self.logger.info("Exit code: {}".format(exit_code))
            self.logger.info("Total FPS is {}".format(total_fps))
            self.logger.info("Per Stream FPS is {}".format(per_stream_fps))
            self.logger.info("Num of Streams is {}".format(num_streams))

            return PipelineRunResult(
                total_fps=total_fps,
                per_stream_fps=per_stream_fps,
                num_streams=num_streams,
            )

        except Exception as e:
            self.logger.error(f"Pipeline execution error: {e}")
            raise

    def cancel(self):
        """Cancel the currently running pipeline."""
        self.cancelled = True

    def is_cancelled(self) -> bool:
        """Check if the pipeline run has been cancelled."""
        return self.cancelled

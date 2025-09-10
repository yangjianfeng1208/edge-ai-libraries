import logging
import math
import os
from pathlib import Path
import struct

from gstpipeline import GstPipeline
from pipelines._common.common import PipelineElementsSelector
from utils import UINT8_DTYPE_SIZE, VIDEO_STREAM_META_PATH, is_yolov10_model

logger = logging.getLogger("smartnvr")


class SmartNVRPipeline(GstPipeline):
    def __init__(self):
        super().__init__()

        self._diagram = Path(os.path.dirname(__file__)) / "diagram.png"

        self._bounding_boxes = [
            (325, 108, 445, 168, "Inference", "Object Detection"),
        ]

        self._sink = "sink_{id}::xpos={xpos} sink_{id}::ypos={ypos} sink_{id}::alpha=1 "

        # Add shmsink for live-streaming (shared memory)
        self._shmsink = (
            "shmsink socket-path=/tmp/shared_memory/video_stream "
            "wait-for-connection=false "
            "sync=true "
            "name=shmsink0 "
        )

        # Use tee to split output to both file and live stream
        self._compositor_with_tee = (
            "{compositor} "
            "  name=comp "
            "  {sinks} ! tee name=livetee "
            "livetee. ! queue2 ! {encoder} ! h264parse ! mp4mux ! filesink location={VIDEO_OUTPUT_PATH} async=false "
            "livetee. ! queue2 ! videoconvert ! video/x-raw,format=BGR,width={output_width},height={output_height} ! {shmsink} "
        )

        self._compositor = (
            "{compositor} "
            "  name=comp "
            "  {sinks} ! "
            "{encoder} ! "
            "h264parse ! "
            "mp4mux ! "
            "filesink "
            "  location={VIDEO_OUTPUT_PATH} async=false "
        )

        self._recording_stream = (
            "filesrc "
            "  location={VIDEO_PATH} ! "
            "qtdemux ! "
            "h264parse ! "
            "tee name=t{id} ! "
            "queue2 ! "
            "mp4mux ! "
            "filesink "
            "  location=/tmp/stream{id}.mp4 "
            "t{id}. ! "
            "queue2 ! "
            "{decoder} ! "
            "gvafpscounter starting-frame=500 ! "
        )

        self._inference_stream_decode_detect_track = (
            # Input
            "filesrc "
            "  location={VIDEO_PATH} ! "
            "qtdemux ! "
            "h264parse ! "
            "tee name=t{id} ! "
            "queue2 ! "
            "mp4mux ! "
            "filesink "
            "  location=/tmp/stream{id}.mp4 "
            "t{id}. ! "
            "queue2 ! "
            # Decoder
            "{decoder} ! "
            # Detection
            "gvafpscounter starting-frame=500 ! "
            "gvadetect "
            "  {detection_model_config} "
            "  model-instance-id=detect0 "
            "  pre-process-backend={object_detection_pre_process_backend} "
            "  device={object_detection_device} "
            "  batch-size={object_detection_batch_size} "
            "  inference-interval={object_detection_inference_interval} "
            "  {ie_config_parameter} "
            "  nireq={object_detection_nireq} ! "
            "queue2 ! "
            "gvatrack "
            "  tracking-type={tracking_type} ! "
            "queue2 ! "
        )

        self._inference_stream_classify = (
            "gvaclassify "
            "  {classification_model_config} "
            "  model-instance-id=classify0 "
            "  pre-process-backend={object_classification_pre_process_backend} "
            "  device={object_classification_device} "
            "  batch-size={object_classification_batch_size} "
            "  inference-interval={object_classification_inference_interval} "
            "  nireq={object_classification_nireq} "
            "  reclassify-interval={object_classification_reclassify_interval} ! "
            "queue2 ! "
        )

        self._inference_stream_metadata_processing = (
            "gvametaconvert "
            "  format=json "
            "  json-indent=4 "
            "  source={VIDEO_PATH} ! "
            "gvametapublish "
            "  method=file "
            "  file-path=/dev/null ! "
        )

        self._sink_to_compositor = (
            "queue2 "
            "  max-size-buffers={max_size_buffers} "
            "  max-size-bytes=0 "
            "  max-size-time=0 ! "
            "{postprocessing} ! "
            "video/x-raw,width=640,height=360 ! "
            "comp.sink_{id} "
        )

    def evaluate(
        self,
        constants: dict,
        parameters: dict,
        regular_channels: int,
        inference_channels: int,
        elements: list | None = None,
    ) -> str:
        if elements is None:
            elements = []

        # Set pre-process backend for object detection
        parameters["object_detection_pre_process_backend"] = (
            "opencv"
            if parameters["object_detection_device"] in ["CPU", "NPU"]
            else "va-surface-sharing"
        )

        # Set pre-process backend for object classification
        parameters["object_classification_pre_process_backend"] = (
            "opencv"
            if parameters["object_classification_device"] in ["CPU", "NPU"]
            else "va-surface-sharing"
        )

        # Use PipelineElementsSelector for element selection
        selector = PipelineElementsSelector(parameters, elements)
        _compositor_element = selector.compositor_element()
        _encoder_element = selector.encoder_element()
        _decoder_element = selector.decoder_element()
        _postprocessing_element = selector.postprocessing_element()

        # If any of the essential elements is not found, log an error and return an empty string
        if not all(
            [
                _compositor_element,
                _encoder_element,
                _decoder_element,
                _postprocessing_element,
            ]
        ):
            logger.error("Could not find all necessary elements for the pipeline.")
            logger.error(
                f"Compositor: {_compositor_element}, Encoder: {_encoder_element}, Decoder: {_decoder_element}, Postprocessing: {_postprocessing_element}"
            )
            return ""
        else:
            logger.info(
                f"Using pipeline elements - Compositor: {_compositor_element}, Encoder: {_encoder_element}, Decoder: {_decoder_element}, Postprocessing: {_postprocessing_element}"
            )

        # Create the streams
        streams = ""

        # Handle object detection parameters and constants
        detection_model_config = (
            f"model={constants['OBJECT_DETECTION_MODEL_PATH']} "
            f"model-proc={constants['OBJECT_DETECTION_MODEL_PROC']} "
        )

        if not constants["OBJECT_DETECTION_MODEL_PROC"]:
            detection_model_config = (
                f"model={constants['OBJECT_DETECTION_MODEL_PATH']} "
            )

        # Set inference config parameter for GPU if using YOLOv10
        ie_config_parameter = ""
        if parameters.get("object_detection_device", "").startswith(
            "GPU"
        ) and is_yolov10_model(constants["OBJECT_DETECTION_MODEL_PATH"]):
            ie_config_parameter = "ie-config=GPU_DISABLE_WINOGRAD_CONVOLUTION=YES"

        # Compute total number of channels
        channels = regular_channels + inference_channels

        # Create a sink for each channel
        sinks = ""
        grid_size = math.ceil(math.sqrt(channels))
        for i in range(channels):
            xpos = 640 * (i % grid_size)
            ypos = 360 * (i // grid_size)
            sinks += self._sink.format(id=i, xpos=xpos, ypos=ypos)

        # Prepare shmsink and meta if live_preview_enabled
        output_width = 0
        output_height = 0
        if parameters["live_preview_enabled"]:
            # Calculate output video size for grid layout to ensure same resolution for shmsink and output file
            output_width = 640 * grid_size
            output_height = 360 * (
                (channels + grid_size - 1) // grid_size
            )  # ceil(channels / grid_size)

            # Write meta file for live preview
            try:
                os.makedirs("/tmp/shared_memory", exist_ok=True)
                with open(VIDEO_STREAM_META_PATH, "wb") as f:
                    # height=output_height, width=output_width, dtype_size=UINT8_DTYPE_SIZE (uint8)
                    f.write(
                        struct.pack(
                            "III", output_height, output_width, UINT8_DTYPE_SIZE
                        )
                    )
            except Exception as e:
                logger.warning(f"Could not write shared memory meta file: {e}")

        # Handle inference channels
        for i in range(inference_channels):
            streams += self._inference_stream_decode_detect_track.format(
                **parameters,
                **constants,
                id=i,
                decoder=_decoder_element,
                detection_model_config=detection_model_config,
                ie_config_parameter=ie_config_parameter,
            )

            # Handle object classification parameters and constants
            # Do this only if the object classification model is not disabled or the device is not disabled
            if not (
                constants["OBJECT_CLASSIFICATION_MODEL_PATH"] == "Disabled"
                or parameters["object_classification_device"] == "Disabled"
            ):
                # Set model config for object classification
                classification_model_config = (
                    f"model={constants['OBJECT_CLASSIFICATION_MODEL_PATH']} "
                    f"model-proc={constants['OBJECT_CLASSIFICATION_MODEL_PROC']} "
                )

                if not constants["OBJECT_CLASSIFICATION_MODEL_PROC"]:
                    classification_model_config = (
                        f"model={constants['OBJECT_CLASSIFICATION_MODEL_PATH']} "
                    )

                streams += self._inference_stream_classify.format(
                    **parameters,
                    **constants,
                    id=i,
                    classification_model_config=classification_model_config,
                )

            # Overlay inference results on the inferred video if enabled
            if parameters["pipeline_watermark_enabled"]:
                streams += "gvawatermark ! "

            # Metadata processing and publishing
            streams += self._inference_stream_metadata_processing.format(
                **parameters,
                **constants,
                id=i,
            )

            # sink to compositor or fake sink depending on the compose flag
            streams += self._sink_to_compositor.format(
                **parameters,
                **constants,
                id=i,
                postprocessing=_postprocessing_element,
                max_size_buffers=0,
            )
        # Handle regular channels
        for i in range(inference_channels, channels):
            streams += self._recording_stream.format(
                **parameters,
                **constants,
                id=i,
                decoder=_decoder_element,
            )
            # sink to compositor or fake sink depending on the compose flag
            streams += self._sink_to_compositor.format(
                **parameters,
                **constants,
                id=i,
                postprocessing=_postprocessing_element,
                max_size_buffers=1,
            )
        # Compose pipeline depending on live_preview_enabled
        if parameters["live_preview_enabled"]:
            # Always produce both file and live stream outputs
            streams = (
                self._compositor_with_tee.format(
                    **constants,
                    sinks=sinks,
                    encoder=_encoder_element,
                    compositor=_compositor_element,
                    shmsink=self._shmsink,
                    output_width=output_width,
                    output_height=output_height,
                )
                + streams
            )
        else:
            # Prepend the compositor
            streams = (
                self._compositor.format(
                    **constants,
                    sinks=sinks,
                    encoder=_encoder_element,
                    compositor=_compositor_element,
                )
                + streams
            )

        # Evaluate the pipeline
        return "gst-launch-1.0 -q " + streams

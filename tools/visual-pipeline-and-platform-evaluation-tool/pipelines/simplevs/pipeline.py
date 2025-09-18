import logging
import os
from pathlib import Path
import struct

from gstpipeline import GstPipeline
from pipelines.common import (
    PipelineElementsSelector,
    PipelineElementSelectionInstructions,
    VAAPI_SUFFIX_PLACEHOLDER,
    GPU_0,
    GPU_N,
    OTHER,
)
from utils import (
    get_video_resolution,
    UINT8_DTYPE_SIZE,
    VIDEO_STREAM_META_PATH,
    is_yolov10_model,
)

logger = logging.getLogger("simplevs")


class SimpleVideoStructurizationPipeline(GstPipeline):
    def __init__(self):
        super().__init__()

        self._diagram = Path(os.path.dirname(__file__)) / "diagram.png"

        self._bounding_boxes = [
            (330, 110, 445, 170, "Inference", "Object Detection"),
        ]

        # Add shmsink for live-streaming (shared memory)
        self._shmsink = (
            "shmsink socket-path=/tmp/shared_memory/video_stream "
            "wait-for-connection=false "
            "sync=true "
            "name=shmsink0 "
        )

        # shmsink branch for live preview (BGR format), width/height will be formatted in evaluate
        self._shmsink_branch = (
            "tee name=livetee "
            "livetee. ! queue2 ! {encoder} ! h264parse ! mp4mux ! filesink location={VIDEO_OUTPUT_PATH} async=false "
            "livetee. ! queue2 ! videoconvert ! video/x-raw,format=BGR,width={output_width},height={output_height} ! {shmsink} "
        )

        self._inference_stream_decode_detect_track = (
            # Input
            "filesrc "
            "  location={VIDEO_PATH} ! "
            "qtdemux ! "
            "h264parse ! "
            # Decoder
            "{decoder} ! "
            "{postprocessing}"  # postprocessing is optional, if present it will have a trailing " ! "
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

        self._inference_output_stream = (
            "{encoder} ! h264parse ! mp4mux ! filesink   location={VIDEO_OUTPUT_PATH} "
        )

        self._selector = PipelineElementsSelector(
            PipelineElementSelectionInstructions(
                encoder={
                    GPU_0: [
                        ("vah264lpenc", "vah264lpenc"),
                        ("vah264enc", "vah264enc"),
                    ],
                    GPU_N: [
                        (
                            f"varenderD{VAAPI_SUFFIX_PLACEHOLDER}h264lpenc",
                            f"varenderD{VAAPI_SUFFIX_PLACEHOLDER}h264lpenc",
                        ),
                        (
                            f"varenderD{VAAPI_SUFFIX_PLACEHOLDER}h264enc",
                            f"varenderD{VAAPI_SUFFIX_PLACEHOLDER}h264enc",
                        ),
                    ],
                    OTHER: [
                        ("x264enc", "x264enc bitrate=16000 speed-preset=superfast"),
                    ],
                },
                decoder={
                    GPU_0: [
                        (
                            "vaapidecodebin",
                            "vaapidecodebin",
                        ),
                    ],
                    GPU_N: [
                        (
                            "vaapidecodebin",
                            "vaapidecodebin",
                        ),
                    ],
                    OTHER: [
                        ("decodebin", "decodebin"),
                    ],
                },
                postprocessing={
                    GPU_0: [
                        (
                            "vapostproc",
                            "vapostproc ! video/x-raw\\(memory:VAMemory\\) ! ",
                        ),
                    ],
                    GPU_N: [
                        (
                            f"varenderD{VAAPI_SUFFIX_PLACEHOLDER}postproc",
                            f"varenderD{VAAPI_SUFFIX_PLACEHOLDER}postproc ! video/x-raw\\(memory:VAMemory\\) ! ",
                        ),
                    ],
                    OTHER: [
                        ("", ""),  # force empty string if no postprocessing is needed
                    ],
                },
            )
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
        (
            _,
            _encoder_element,
            _decoder_element,
            _postprocessing_element,
        ) = self._selector.select_elements(parameters, elements)

        # If any of the essential elements is not found, log an error and return an empty string
        # _postprocessing_element is optional, so don't include it in the check
        if not all(
            [
                _encoder_element,
                _decoder_element,
            ]
        ):
            logger.error("Could not find all necessary elements for the pipeline.")
            logger.error(
                f"Encoder: {_encoder_element}, Decoder: {_decoder_element}, Postprocessing: {_postprocessing_element}"
            )
            return ""
        else:
            logger.debug(
                f"Using pipeline elements - Encoder: {_encoder_element}, Decoder: {_decoder_element}, Postprocessing: {_postprocessing_element}"
            )

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

        # Prepare shmsink and meta if live_preview_enabled
        output_width = 0
        output_height = 0
        if parameters["live_preview_enabled"]:
            # Get resolution using get_video_resolution
            video_path = constants.get("VIDEO_PATH", "")
            output_width, output_height = get_video_resolution(video_path)

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

        # Create the streams
        streams = ""

        # Handle inference channels
        for i in range(inference_channels):
            streams += self._inference_stream_decode_detect_track.format(
                **parameters,
                **constants,
                decoder=_decoder_element,
                postprocessing=_postprocessing_element,
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
                    classification_model_config=classification_model_config,
                )

            # Overlay inference results on the inferred video if enabled
            if (
                parameters["pipeline_watermark_enabled"]
                and parameters["pipeline_video_enabled"]
            ):
                streams += "gvawatermark ! "

            # Metadata processing and publishing
            streams += self._inference_stream_metadata_processing.format(
                **parameters,
                **constants,
            )

            # Use video output for the first inference channel if enabled, otherwise use fakesink
            if i == 0 and (
                parameters["pipeline_video_enabled"]
                or parameters["live_preview_enabled"]
            ):
                if parameters["live_preview_enabled"]:
                    # Use tee to split to both file and shmsink, fill in width/height here
                    streams += self._shmsink_branch.format(
                        **constants,
                        encoder=_encoder_element,
                        output_width=output_width,
                        output_height=output_height,
                        shmsink=self._shmsink,
                    )
                else:
                    streams += self._inference_output_stream.format(
                        **constants, encoder=_encoder_element
                    )
            else:
                streams += "fakesink "

        # Evaluate the pipeline
        return "gst-launch-1.0 -q " + streams

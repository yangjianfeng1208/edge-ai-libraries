from typing import Dict, Any, List, Optional
from pydantic import BaseModel, Field
from enum import Enum


# # Enums based on OpenAPI schema
class PipelineType(str, Enum):
    GSTREAMER = "GStreamer"
    FFMPEG = "FFmpeg"


class PipelineSource(str, Enum):
    PREDEFINED = "PREDEFINED"
    USER_CREATED = "USER_CREATED"


class TestJobState(str, Enum):
    RUNNING = "RUNNING"
    COMPLETED = "COMPLETED"
    ERROR = "ERROR"
    ABORTED = "ABORTED"


class OptimizationJobState(str, Enum):
    RUNNING = "RUNNING"
    COMPLETED = "COMPLETED"
    ERROR = "ERROR"
    ABORTED = "ABORTED"


class ValidationJobState(str, Enum):
    RUNNING = "RUNNING"
    COMPLETED = "COMPLETED"
    ERROR = "ERROR"
    ABORTED = "ABORTED"


class SourceType(str, Enum):
    URI = "uri"
    GST = "gst"


class DeviceType(str, Enum):
    DISCRETE = "DISCRETE"
    INTEGRATED = "INTEGRATED"


class DeviceFamily(str, Enum):
    CPU = "CPU"
    GPU = "GPU"
    NPU = "NPU"


class ModelCategory(str, Enum):
    CLASSIFICATION = "classification"
    DETECTION = "detection"


class OptimizationType(str, Enum):
    PREPROCESS = "preprocess"
    OPTIMIZE = "optimize"


# Define minimal models based on schema references
class Source(BaseModel):
    type: SourceType
    uri: Optional[str]


class Node(BaseModel):
    id: str
    type: str
    data: Dict[str, str]


class Edge(BaseModel):
    id: str
    source: str
    target: str


class MessageResponse(BaseModel):
    message: str


class PipelineDescription(BaseModel):
    pipeline_description: str


class PipelineGraph(BaseModel):
    nodes: list[Node]
    edges: list[Edge]


class PipelineParameters(BaseModel):
    default: Optional[Dict[str, Any]]


class PipelinePerformanceSpec(BaseModel):
    id: str
    streams: int = Field(default=1, ge=0)


class PipelineDensitySpec(BaseModel):
    id: str
    stream_rate: int = Field(default=100, ge=0)


class Pipeline(BaseModel):
    id: str
    name: str
    version: int
    description: str
    source: PipelineSource
    type: PipelineType
    pipeline_graph: PipelineGraph
    parameters: Optional[PipelineParameters]


class PipelineDefinition(BaseModel):
    name: str
    version: int = Field(default=1, ge=1)
    description: str
    source: PipelineSource = PipelineSource.USER_CREATED
    type: PipelineType
    pipeline_description: str
    parameters: Optional[PipelineParameters]


class PipelineValidation(BaseModel):
    type: PipelineType = PipelineType.GSTREAMER
    pipeline_graph: PipelineGraph
    parameters: Optional[Dict[str, Any]] = Field(
        default=None, examples=[{"max-runtime": 10}]
    )


class ValidationJobResponse(BaseModel):
    job_id: str


class PipelineRequestOptimize(BaseModel):
    type: OptimizationType
    parameters: Optional[Dict[str, Any]]


class EncoderDeviceConfig(BaseModel):
    device_name: str = Field(
        default="GPU",
        description="Name of the encoder device (e.g., 'GPU', 'CPU', 'NPU')",
        examples=["GPU", "CPU", "NPU"],
    )
    gpu_id: Optional[int] = Field(
        default=None,
        description="GPU device index (only applicable when device_name indicates a GPU)",
        examples=[0, 1],
    )


class VideoOutputConfig(BaseModel):
    enabled: bool = Field(
        default=False, description="Flag to enable or disable video output generation."
    )
    encoder_device: EncoderDeviceConfig = Field(
        default=EncoderDeviceConfig(device_name="GPU", gpu_id=0),
        description="Encoder device configuration (only applicable when video output is enabled).",
        examples=[{"device_name": "GPU", "gpu_id": 0}],
    )


class PerformanceTestSpec(BaseModel):
    pipeline_performance_specs: list[PipelinePerformanceSpec]
    video_output: VideoOutputConfig = Field(
        default=VideoOutputConfig(
            enabled=False,
            encoder_device=EncoderDeviceConfig(device_name="GPU", gpu_id=0),
        ),
        description="Video output configuration.",
        examples=[
            {"enabled": False, "encoder_device": {"device_name": "GPU", "gpu_id": 0}}
        ],
    )


class DensityTestSpec(BaseModel):
    fps_floor: int = Field(ge=0, examples=[30])
    pipeline_density_specs: list[PipelineDensitySpec]
    video_output: VideoOutputConfig = Field(
        default=VideoOutputConfig(
            enabled=False,
            encoder_device=EncoderDeviceConfig(device_name="GPU", gpu_id=0),
        ),
        description="Video output configuration.",
        examples=[
            {"enabled": False, "encoder_device": {"device_name": "GPU", "gpu_id": 0}}
        ],
    )


class TestJobResponse(BaseModel):
    job_id: str


class TestsJobStatus(BaseModel):
    id: str
    start_time: int
    elapsed_time: int
    state: TestJobState
    total_fps: Optional[float]
    per_stream_fps: Optional[float]
    total_streams: Optional[int]
    streams_per_pipeline: Optional[List[PipelinePerformanceSpec]]
    video_output_paths: Optional[Dict[str, List[str]]]
    error_message: Optional[str]


class PerformanceJobStatus(TestsJobStatus):
    pass


class DensityJobStatus(TestsJobStatus):
    pass


class PerformanceJobSummary(BaseModel):
    id: str
    request: PerformanceTestSpec


class DensityJobSummary(BaseModel):
    id: str
    request: DensityTestSpec


class OptimizationJobResponse(BaseModel):
    job_id: str


class OptimizationJobStatus(BaseModel):
    id: str
    type: Optional[OptimizationType]
    start_time: int
    elapsed_time: int
    state: OptimizationJobState
    total_fps: Optional[float]
    original_pipeline_graph: PipelineGraph
    optimized_pipeline_graph: Optional[PipelineGraph]
    original_pipeline_description: str
    optimized_pipeline_description: Optional[str]
    error_message: Optional[str]


class OptimizationJobSummary(BaseModel):
    id: str
    request: PipelineRequestOptimize


class ValidationJobStatus(BaseModel):
    id: str
    start_time: int
    elapsed_time: int
    state: ValidationJobState
    is_valid: Optional[bool]
    error_message: Optional[List[str]]


class ValidationJobSummary(BaseModel):
    id: str
    request: PipelineValidation


class Device(BaseModel):
    device_name: str
    full_device_name: str
    device_type: DeviceType
    device_family: DeviceFamily
    gpu_id: Optional[int]


class Model(BaseModel):
    name: str
    display_name: str
    category: Optional[ModelCategory]
    precision: Optional[str]


class MetricSample(BaseModel):
    name: str
    description: str
    timestamp: int
    value: float


class Video(BaseModel):
    filename: str
    width: int
    height: int
    fps: float
    frame_count: int
    codec: str
    duration: float

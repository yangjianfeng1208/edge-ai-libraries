from typing import Dict, Any, List, Optional
from pydantic import BaseModel
from enum import Enum


# # Enums based on OpenAPI schema
class PipelineType(str, Enum):
    GSTREAMER = "GStreamer"
    FFMPEG = "FFmpeg"


class PipelineInstanceState(str, Enum):
    RUNNING = "RUNNING"
    COMPLETED = "COMPLETED"
    ERROR = "ERROR"
    ABORTED = "ABORTED"


class OptimizationJobState(str, Enum):
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


class PipelineDescription(BaseModel):
    pipeline_description: str


class Node(BaseModel):
    id: str
    type: str
    data: Dict[str, str]


class Edge(BaseModel):
    id: str
    source: str
    target: str


class PipelineGraph(BaseModel):
    nodes: list[Node]
    edges: list[Edge]


class MessageResponse(BaseModel):
    message: str


class PipelineParameters(BaseModel):
    default: Optional[Dict[str, Any]]


class PipelineRunSpec(BaseModel):
    name: str
    version: str
    streams: int = 1


class PipelineBenchmarkSpec(BaseModel):
    name: str
    version: str
    stream_rate: int = 100


class Pipeline(BaseModel):
    name: str
    version: str
    description: str
    type: PipelineType
    pipeline_graph: PipelineGraph
    parameters: Optional[PipelineParameters]


class PipelineDefinition(BaseModel):
    name: str
    version: str
    description: str
    type: PipelineType
    pipeline_description: str
    parameters: Optional[PipelineParameters]


class PipelineValidation(BaseModel):
    type: PipelineType
    pipeline_description: str
    parameters: Optional[PipelineParameters]


class PipelineRequestRun(BaseModel):
    pipeline_run_specs: list[PipelineRunSpec]


class PipelineRequestBenchmark(BaseModel):
    fps_floor: int = 30
    pipeline_benchmark_specs: list[PipelineBenchmarkSpec]


class PipelineRequestOptimize(BaseModel):
    type: OptimizationType
    parameters: Optional[Dict[str, Any]]


class PipelineInstanceResponse(BaseModel):
    instance_id: str


class PipelineInstanceStatus(BaseModel):
    id: str
    start_time: int
    elapsed_time: int
    state: PipelineInstanceState
    total_fps: Optional[float]
    per_stream_fps: Optional[float]
    total_streams: Optional[int]
    streams_per_pipeline: Optional[List[PipelineRunSpec]]
    error_message: Optional[str]


class PipelineInstanceSummary(BaseModel):
    id: str
    request: PipelineRequestRun | PipelineRequestBenchmark
    type: str


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

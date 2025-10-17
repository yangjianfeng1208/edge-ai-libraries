import os
import tempfile
from typing import List
from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse
from uuid import UUID

import api.api_schemas as schemas
from gstpipeline import PipelineLoader
from optimize import PipelineOptimizer
from explore import GstInspector
from utils import download_file, replace_file_path
from benchmark import Benchmark

TEMP_DIR = tempfile.gettempdir()

router = APIRouter()
gst_inspector = GstInspector()


@router.get("", response_model=List[schemas.Pipeline])
def get_pipelines():
    pipeline_infos = []
    for pipeline in PipelineLoader.list():
        pipeline_gst, config = PipelineLoader.load(pipeline)
        pipeline_infos.append(
            schemas.Pipeline(
                name=config.get("name", "Unnamed Pipeline"),
                version=config.get("version", "0.0.1"),
                description=config.get("definition", config.get("description", "")),
                type=schemas.PipelineType.GSTREAMER,
                parameters=schemas.PipelineParameters(
                    default={
                        "launch_string": pipeline_gst.get_default_gst_launch(
                            elements=gst_inspector.get_elements()
                        )
                    }
                ),
            )
        )
    return pipeline_infos


@router.post(
    "",
    status_code=201,
    responses={
        201: {
            "description": "Pipeline created",
            "headers": {
                "Location": {
                    "description": "URL of the created pipeline",
                    "schema": {"type": "string"},
                }
            },
        },
        500: {"description": "Internal server error"},
    },
)
def create_pipeline(body: schemas.PipelineDefinition):
    """Create a custom pipeline from a launch string."""
    try:
        # TODO: Validate the launch string and create pipeline

        location = f"/pipelines/{body.name}/{body.version}"
        return JSONResponse(
            content="Pipeline created",
            status_code=201,
            headers={"Location": location},
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Failed to create pipeline: {str(e)}"
        )


@router.post(
    "/validate",
    status_code=200,
    responses={
        200: {"description": "Pipeline is valid"},
        400: {"description": "Invalid launch string"},
    },
)
def validate_pipeline(body: schemas.PipelineValidation):
    """Validate launch string pipeline."""
    # TODO: Implement actual validation logic
    return JSONResponse(content={"message": "Pipeline valid"}, status_code=200)


@router.post("/{name}/{version}/run")
def run_pipeline(name: str, version: str, body: schemas.PipelineRequestRun):
    # Download the pipeline recording file
    file_name = os.path.basename(str(body.source.uri))
    file_path = download_file(
        body.source.uri,
        file_name,
    )
    # Replace file path in launch string if needed
    launch_string = replace_file_path(body.parameters.launch_string, file_path)

    # Initialize pipeline object from launch string
    gst_pipeline, config = PipelineLoader.load_from_launch_string(
        launch_string, name=name
    )

    inferencing_channels = body.parameters.inferencing_channels
    recording_channels = body.parameters.recording_channels

    if recording_channels + inferencing_channels == 0:
        return {"error": "At least one channel must be enabled"}

    # TODO: Enable live preview when implemented
    param_grid = {"live_preview_enabled": ["false"]}

    optimizer = PipelineOptimizer(
        pipeline=gst_pipeline,
        param_grid=param_grid,
        channels=(recording_channels, inferencing_channels),
        elements=gst_inspector.get_elements(),
    )

    optimizer.run_without_live_preview()

    best_result = optimizer.evaluate()
    if best_result is None:
        best_result_message = "No valid result was returned by the optimizer."
    else:
        best_result_message = (
            f"Total FPS: {best_result.total_fps:.2f}, "
            f"Per Stream FPS: {best_result.per_stream_fps:.2f}"
        )

    return best_result_message


@router.post("/{name}/{version}/benchmark")
def benchmark_pipeline(name: str, version: str, body: schemas.PipelineRequestBenchmark):
    # Download the pipeline recording file
    file_name = os.path.basename(str(body.source.uri))
    file_path = download_file(
        body.source.uri,
        file_name,
    )
    # Replace file path in launch string if needed
    launch_string = replace_file_path(body.parameters.launch_string, file_path)

    # Initialize pipeline object from launch string
    gst_pipeline, config = PipelineLoader.load_from_launch_string(
        launch_string, name=name
    )

    # Disable live preview for benchmarking
    param_grid = {"live_preview_enabled": ["false"]}

    # Initialize the benchmark class
    bm = Benchmark(
        pipeline_cls=gst_pipeline,
        fps_floor=body.parameters.fps_floor,
        rate=body.parameters.ai_stream_rate,
        parameters=param_grid,
        elements=gst_inspector.get_elements(),
    )

    # Run the benchmark
    s, ai, non_ai, fps = bm.run()

    # Return results
    try:
        result = config["parameters"]["benchmark"]["result_format"]
    except KeyError:
        result = "Best Config: {s} streams ({ai} AI, {non_ai} non_AI) -> {fps:.2f} FPS"

    return result.format(s=s, ai=ai, non_ai=non_ai, fps=fps)


@router.post("/{name}/{version}/optimize")
def optimize_pipeline(
    name: str, version: str, request: schemas.PipelineRequestOptimize
):
    return {"message": "Optimization started"}


@router.delete(
    "/{name}/{version}",
    status_code=200,
    responses={
        200: {"description": "Pipeline deleted"},
        400: {"description": "Cannot delete pipeline"},
    },
)
def delete_pipeline(name: str, version: str):
    return {"message": "Pipeline deleted"}


@router.get("/status", response_model=List[schemas.PipelineInstanceStatus])
def get_pipeline_status():
    return []


@router.delete("/{instance_id}", response_model=List[schemas.PipelineInstanceStatus])
def stop_pipeline_instance(instance_id: UUID):
    return []


@router.get("/{instance_id}", response_model=schemas.PipelineInstanceSummary)
def get_pipeline_summary(instance_id: UUID):
    return []


@router.get("/{instance_id}/status", response_model=schemas.PipelineInstanceStatus)
def get_pipeline_instance_status(instance_id: UUID):
    return []

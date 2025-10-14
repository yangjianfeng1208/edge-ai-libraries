import os
import logging
import tempfile
from typing import List
from fastapi import APIRouter, HTTPException
from fastapi.responses import JSONResponse
from uuid import UUID

import api_schemas as schemas
from gstpipeline import PipelineLoader
from optimize import PipelineOptimizer
from explore import GstInspector
from utils import prepare_video_and_constants, download_file
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
    is_valid, message = validate_launch_string(body.launch_string)
    if is_valid:
        return JSONResponse(content={"message": "Pipeline valid"}, status_code=200)
    else:
        raise HTTPException(status_code=400, detail=message)

@router.post("/{name}/{version}/run")
def run_pipeline(name: str, version: str, body: schemas.PipelineRequestRun):
    parameters = body.parameters or {}

    gst_pipeline, config = PipelineLoader.load_from_launch_string(
        parameters["launch_string"], name=name
    )

    # Download the pipeline recording files if using built-in config
    if "recording" in config:
        download_file(
            config["recording"]["url"],
            config["recording"]["filename"],
        )
        input_video_path = os.path.join(TEMP_DIR, config["recording"]["filename"])
    else:
        file_name = os.path.basename(body.source.uri)
        download_file(
            body.source.uri,
            file_name,
        )
        # Use the source URI from the request
        input_video_path = os.path.join(TEMP_DIR, file_name)

    recording_channels = 0
    inferencing_channels = parameters.get("inferencing_channels", 1)

    if recording_channels + inferencing_channels == 0:
        return {"error": "At least one channel must be enabled"}

    param_grid = {"live_preview_enabled": [False]}

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
    dir = "smartnvr"  # This should be mapped from name/version in a real implementation
    gst_pipeline, config = PipelineLoader.load(dir)

    # Download the pipeline recording files if using built-in config
    if "recording" in config:
        download_file(
            config["recording"]["url"],
            config["recording"]["filename"],
        )
        input_video_path = os.path.join(TEMP_DIR, config["recording"]["filename"])
    else:
        file_name = os.path.basename(body.source.uri)
        download_file(
            body.source.uri,
            file_name,
        )
        # Use the source URI from the request
        input_video_path = os.path.join(TEMP_DIR, file_name)

    # Prepare parameters, ensuring input video is set
    parameters = body.parameters or {}
    parameters["input_video_player"] = input_video_path

    try:
        video_output_path, constants, param_grid = prepare_video_and_constants(
            **parameters
        )
    except ValueError as e:
        return {"error": str(e)}

    logging.info(f"Constants: {constants}")
    logging.info(f"param_grid: {param_grid}")

    # Initialize the benchmark class
    bm = Benchmark(
        pipeline_cls=gst_pipeline,
        fps_floor=parameters.get("fps_floor"),
        rate=parameters.get("ai_stream_rate"),
        parameters=param_grid,
        constants=constants,
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
    return schemas.PipelineInstanceSummary(id=0, type="type")


@router.get("/{instance_id}/status", response_model=schemas.PipelineInstanceStatus)
def get_pipeline_instance_status(instance_id: UUID):
    return schemas.PipelineInstanceStatus(id=0, state="RUNNING")

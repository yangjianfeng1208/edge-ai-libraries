import tempfile
from typing import List
from fastapi import APIRouter
from fastapi.responses import JSONResponse

import api.api_schemas as schemas
from managers.pipeline_manager import PipelineManager
from managers.instance_manager import InstanceManager

TEMP_DIR = tempfile.gettempdir()

router = APIRouter()
pipeline_manager = PipelineManager()
instance_manager = InstanceManager()


@router.get("", operation_id="get_pipelines", response_model=List[schemas.Pipeline])
def get_pipelines():
    return pipeline_manager.get_pipelines()


@router.post(
    "",
    operation_id="create_pipeline",
    status_code=201,
    responses={
        201: {
            "description": "Pipeline created",
            "model": schemas.MessageResponse,
            "headers": {
                "Location": {
                    "description": "URL of the created pipeline",
                    "schema": {"type": "string"},
                }
            },
        },
        400: {
            "description": "Pipeline already exists",
            "model": schemas.MessageResponse,
        },
        500: {"description": "Internal server error", "model": schemas.MessageResponse},
    },
)
def create_pipeline(body: schemas.PipelineDefinition):
    """Create a custom pipeline from a launch string."""
    # TODO: Validate the launch string
    try:
        pipeline_manager.add_pipeline(body)

        return JSONResponse(
            content=schemas.MessageResponse(message="Pipeline created").model_dump(),
            status_code=201,
            headers={"Location": f"/pipelines/{body.name}/{body.version}"},
        )
    except ValueError as e:
        return JSONResponse(
            content=schemas.MessageResponse(message=str(e)).model_dump(),
            status_code=400,
        )
    except Exception as e:
        return JSONResponse(
            content=schemas.MessageResponse(
                message=f"Failed to create pipeline: {str(e)}"
            ).model_dump(),
            status_code=500,
        )


@router.post(
    "/validate",
    operation_id="validate_pipeline",
    responses={
        200: {"description": "Pipeline is valid", "model": schemas.MessageResponse},
        400: {"description": "Invalid launch string", "model": schemas.MessageResponse},
    },
)
def validate_pipeline(body: schemas.PipelineValidation):
    """Validate launch string pipeline."""
    # TODO: Implement actual validation logic
    return schemas.MessageResponse(message="Pipeline is valid")


@router.get(
    "/status",
    operation_id="get_pipeline_statuses",
    response_model=List[schemas.PipelineInstanceStatus],
)
def get_pipeline_statuses():
    """Get status of all pipeline instances."""
    return instance_manager.get_all_instance_statuses()


@router.get(
    "/{instance_id}",
    operation_id="get_pipeline_instance_summary",
    responses={
        200: {
            "description": "Successful Response",
            "model": schemas.PipelineInstanceSummary,
        },
        404: {"description": "Instance not found", "model": schemas.MessageResponse},
    },
)
def get_instance_summary(instance_id: str):
    """Get summary of a specific pipeline instance."""
    summary = instance_manager.get_instance_summary(instance_id)
    if summary is None:
        return JSONResponse(
            content=schemas.MessageResponse(
                message=f"Instance {instance_id} not found"
            ).model_dump(),
            status_code=404,
        )
    return summary


@router.get(
    "/{instance_id}/status",
    operation_id="get_pipeline_instance_status",
    responses={
        200: {
            "description": "Successful Response",
            "model": schemas.PipelineInstanceStatus,
        },
        404: {"description": "Instance not found", "model": schemas.MessageResponse},
    },
)
def get_pipeline_instance_status(instance_id: str):
    """Get status of a specific pipeline instance."""
    status = instance_manager.get_instance_status(instance_id)
    if status is None:
        return JSONResponse(
            content=schemas.MessageResponse(
                message=f"Instance {instance_id} not found"
            ).model_dump(),
            status_code=404,
        )
    return status


@router.get(
    "/{name}/{version}",
    operation_id="get_pipeline",
    responses={
        200: {"description": "Successful Response", "model": schemas.Pipeline},
        404: {"description": "Pipeline not found", "model": schemas.MessageResponse},
        500: {"description": "Unexpected error", "model": schemas.MessageResponse},
    },
)
def get_pipeline(name: str, version: str):
    try:
        return pipeline_manager.get_pipeline_by_name_and_version(name, version)
    except ValueError as e:
        return JSONResponse(
            content=schemas.MessageResponse(message=str(e)).model_dump(),
            status_code=404,
        )
    except Exception as e:
        return JSONResponse(
            content=schemas.MessageResponse(
                message=f"Unexpected error: {str(e)}"
            ).model_dump(),
            status_code=500,
        )


@router.post(
    "/{name}/{version}",
    operation_id="run_pipeline",
    status_code=202,
    response_model=schemas.PipelineInstanceResponse,
)
def run_pipeline(name: str, version: str, body: schemas.PipelineRequestRun):
    """Run a pipeline."""
    instance_id = instance_manager.run_pipeline(name, version, body)
    return schemas.PipelineInstanceResponse(instance_id=instance_id)


@router.post(
    "/{name}/{version}/benchmark",
    operation_id="benchmark_pipeline",
    status_code=202,
    response_model=schemas.PipelineInstanceResponse,
)
def benchmark_pipeline(name: str, version: str, body: schemas.PipelineRequestBenchmark):
    """Benchmark a pipeline."""
    instance_id = instance_manager.benchmark_pipeline(name, version, body)
    return schemas.PipelineInstanceResponse(instance_id=instance_id)


@router.post("/{name}/{version}/optimize", operation_id="optimize_pipeline")
def optimize_pipeline(
    name: str, version: str, request: schemas.PipelineRequestOptimize
):
    return {"message": "Optimization started"}


@router.delete(
    "/{name}/{version}",
    operation_id="delete_pipeline",
    responses={
        200: {"description": "Pipeline deleted", "model": schemas.MessageResponse},
        400: {
            "description": "Cannot delete pipeline",
            "model": schemas.MessageResponse,
        },
    },
)
def delete_pipeline(name: str, version: str):
    return schemas.MessageResponse(message="Pipeline deleted")


@router.delete(
    "/{instance_id}",
    operation_id="stop_pipeline_instance",
    responses={
        200: {
            "description": "Successful Response",
            "model": schemas.MessageResponse,
        },
        404: {
            "description": "Pipeline instance not found",
            "model": schemas.MessageResponse,
        },
        409: {
            "description": "Pipeline instance not running",
            "model": schemas.MessageResponse,
        },
        500: {
            "description": "Unexpected error",
            "model": schemas.MessageResponse,
        },
    },
)
def stop_pipeline_instance(instance_id: str):
    """Stop a running pipeline instance."""
    success, message = instance_manager.stop_instance(instance_id)
    response = schemas.MessageResponse(message=message)
    if success:
        return response
    if "not found" in message.lower() or "no active runner found" in message.lower():
        return JSONResponse(
            content=response.model_dump(),
            status_code=404,
        )
    if "not running" in message.lower():
        return JSONResponse(
            content=response.model_dump(),
            status_code=409,
        )
    return JSONResponse(
        content=response.model_dump(),
        status_code=500,
    )

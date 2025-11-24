import tempfile
from typing import List
from fastapi import APIRouter
from fastapi.responses import JSONResponse

import api.api_schemas as schemas
from managers.pipeline_manager import get_pipeline_manager
from managers.optimization_manager import get_optimization_manager
from managers.validation_manager import get_validation_manager

TEMP_DIR = tempfile.gettempdir()

router = APIRouter()
pipeline_manager = get_pipeline_manager()
optimization_manager = get_optimization_manager()
validation_manager = get_validation_manager()


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
        # Enforce USER_CREATED source for pipelines created via API
        body.source = schemas.PipelineSource.USER_CREATED
        pipeline = pipeline_manager.add_pipeline(body)

        return JSONResponse(
            content=schemas.MessageResponse(message="Pipeline created").model_dump(),
            status_code=201,
            headers={"Location": f"/pipelines/{pipeline.id}"},
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
        202: {
            "description": "Pipeline validation started",
            "model": schemas.ValidationJobResponse,
        },
        400: {
            "description": "Invalid validation request",
            "model": schemas.MessageResponse,
        },
        500: {"description": "Internal server error", "model": schemas.MessageResponse},
    },
)
def validate_pipeline(body: schemas.PipelineValidation):
    """
    Start an asynchronous validation job for an ad‑hoc pipeline graph.

    The handler:

    * accepts a :class:`PipelineValidation` request,
    * delegates creation of a background validation job to
      :data:`validation_manager`,
    * returns a :class:`ValidationJobResponse` containing the job id.

    Error handling
    --------------
    * Invalid arguments (e.g. non‑positive max‑runtime) result in a
      ``400`` response.
    * Any unexpected exception results in a ``500`` response.
    """
    try:
        job_id = validation_manager.run_validation(body)
        return JSONResponse(
            content=schemas.ValidationJobResponse(job_id=job_id).model_dump(),
            status_code=202,
        )
    except ValueError as e:
        # ValidationManager uses ValueError for user‑level input problems.
        return JSONResponse(
            content=schemas.MessageResponse(message=str(e)).model_dump(),
            status_code=400,
        )
    except Exception as e:
        return JSONResponse(
            content=schemas.MessageResponse(
                message=f"Unexpected error: {str(e)}"
            ).model_dump(),
            status_code=500,
        )


@router.get("", operation_id="get_pipelines", response_model=List[schemas.Pipeline])
def get_pipelines():
    return pipeline_manager.get_pipelines()


@router.get(
    "/{pipeline_id}",
    operation_id="get_pipeline",
    responses={
        200: {"description": "Successful Response", "model": schemas.Pipeline},
        404: {"description": "Pipeline not found", "model": schemas.MessageResponse},
        500: {"description": "Unexpected error", "model": schemas.MessageResponse},
    },
)
def get_pipeline(pipeline_id: str):
    try:
        return pipeline_manager.get_pipeline_by_id(pipeline_id)
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
    "/{pipeline_id}/optimize",
    operation_id="optimize_pipeline",
    responses={
        202: {
            "description": "Pipeline optimization started",
            "model": schemas.OptimizationJobResponse,
        },
        404: {"description": "Pipeline not found", "model": schemas.MessageResponse},
        500: {"description": "Unexpected error", "model": schemas.MessageResponse},
    },
)
def optimize_pipeline(pipeline_id: str, body: schemas.PipelineRequestOptimize):
    """
    Start an asynchronous optimization job for a given pipeline.

    The handler performs the following steps:

    * look up the pipeline identified by ``pipeline_id`` using
      :data:`pipeline_manager`,
    * delegate the optimization request to :data:`optimization_manager`,
      which creates a background job and returns its ``job_id``,
    * wrap the ``job_id`` into :class:`schemas.OptimizationJobResponse`
      and return it with HTTP 202 (Accepted).

    Error handling
    --------------
    * If the pipeline does not exist, a 404 response with
      :class:`schemas.MessageResponse` is returned.
    * Any unexpected exception results in a 500 response with a generic
      error message, while the original exception string is preserved
      for easier debugging.
    """
    try:
        pipeline = pipeline_manager.get_pipeline_by_id(pipeline_id)
        job_id = optimization_manager.run_optimization(pipeline, body)
        return schemas.OptimizationJobResponse(job_id=job_id)
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


@router.delete(
    "/{pipeline_id}",
    operation_id="delete_pipeline",
    responses={
        200: {"description": "Pipeline deleted", "model": schemas.MessageResponse},
        404: {
            "description": "Pipeline not found",
            "model": schemas.MessageResponse,
        },
    },
)
def delete_pipeline(pipeline_id: str):
    """Delete pipeline by ID."""
    try:
        pipeline_manager.delete_pipeline_by_id(pipeline_id)
    except ValueError as e:
        return JSONResponse(
            content=schemas.MessageResponse(message=str(e)).model_dump(),
            status_code=404,
        )
    return schemas.MessageResponse(message="Pipeline deleted")

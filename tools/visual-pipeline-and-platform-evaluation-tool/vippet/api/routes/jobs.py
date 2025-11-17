from typing import List

from fastapi import APIRouter
from fastapi.responses import JSONResponse

import api.api_schemas as schemas
from managers.optimization_manager import get_optimization_manager

router = APIRouter()
optimization_manager = get_optimization_manager()


@router.get(
    "/optimization/status",
    operation_id="get_optimization_statuses",
    response_model=List[schemas.OptimizationJobStatus],
)
def get_optimization_statuses():
    """
    Return the status of all known optimization jobs.

    The response is a list of :class:`OptimizationJobStatus` objects
    describing each job, including:

    * job identifier,
    * current state (RUNNING, COMPLETED, ERROR, ABORTED),
    * timing information (start / elapsed),
    * pipeline graphs and descriptions,
    * optional performance metrics.

    The implementation simply delegates to the global
    :data:`optimization_manager` instance, which performs the actual
    aggregation and logging.
    """
    # Delegate to the manager; FastAPI takes care of serializing the
    # resulting Pydantic models into JSON.
    return optimization_manager.get_all_job_statuses()


@router.get(
    "/optimization/{job_id}",
    operation_id="get_optimization_job_summary",
    responses={
        200: {
            "description": "Successful Response",
            "model": schemas.OptimizationJobSummary,
        },
        404: {
            "description": "Optimization job not found",
            "model": schemas.MessageResponse,
        },
    },
)
def get_optimization_job_summary(job_id: str):
    """
    Return a short summary of a specific optimization job.

    Parameters
    ----------
    job_id:
        The identifier previously returned when the job was created.

    Returns
    -------
    OptimizationJobSummary | JSONResponse
        * On success (job exists): an :class:`OptimizationJobSummary`
          instance containing the job id and the original optimization
          request.
        * On failure (job is unknown): a ``404`` :class:`JSONResponse`
          whose body is a :class:`MessageResponse` explaining that the
          job was not found.
    """
    # Ask the manager for the summary.  It returns None when the job id
    # is unknown, which we map to a 404 HTTP response.
    summary = optimization_manager.get_job_summary(job_id)
    if summary is None:
        # The explicit JSONResponse is used instead of raising HTTPException
        # to mirror the style used by other routes (e.g. pipelines.py) and
        # to fully control the response payload.
        return JSONResponse(
            content=schemas.MessageResponse(
                message=f"Optimization job {job_id} not found"
            ).model_dump(),
            status_code=404,
        )
    return summary


@router.get(
    "/optimization/{job_id}/status",
    operation_id="get_optimization_job_status",
    responses={
        200: {
            "description": "Successful Response",
            "model": schemas.OptimizationJobStatus,
        },
        404: {
            "description": "Optimization job not found",
            "model": schemas.MessageResponse,
        },
    },
)
def get_optimization_job_status(job_id: str):
    """
    Return the detailed status of a specific optimization job.

    Compared to :func:`get_optimization_job_summary`, this endpoint
    exposes the full :class:`OptimizationJobStatus` object, including
    timing, state and (if available) optimization results.

    Parameters
    ----------
    job_id:
        Identifier of the optimization job whose status should be
        retrieved.

    Returns
    -------
    OptimizationJobStatus | JSONResponse
        * On success (job exists): an :class:`OptimizationJobStatus`
          instance describing the current state of the job.
        * On failure (job is unknown): a ``404`` :class:`JSONResponse`
          with a :class:`MessageResponse` body.
    """
    # Query the manager for the job status.  Unknown job ids are mapped
    # to a 404 response, mirroring the behaviour of the summary endpoint.
    status = optimization_manager.get_job_status(job_id)
    if status is None:
        return JSONResponse(
            content=schemas.MessageResponse(
                message=f"Optimization job {job_id} not found"
            ).model_dump(),
            status_code=404,
        )
    return status

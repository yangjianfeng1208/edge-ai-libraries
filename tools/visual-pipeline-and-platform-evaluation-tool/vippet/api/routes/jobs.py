from typing import List

from fastapi import APIRouter
from fastapi.responses import JSONResponse

import api.api_schemas as schemas
from managers.optimization_manager import get_optimization_manager
from managers.tests_manager import get_tests_manager, PerformanceJob, DensityJob
from managers.validation_manager import get_validation_manager

router = APIRouter()
optimization_manager = get_optimization_manager()
tests_manager = get_tests_manager()
validation_manager = get_validation_manager()


def get_job_status_or_404(job_id: str, job_type: str):
    status = tests_manager.get_job_status(job_id)
    if status is None:
        return JSONResponse(
            content=schemas.MessageResponse(
                message=f"{job_type} job {job_id} not found"
            ).model_dump(),
            status_code=404,
        )
    return status


def stop_test_job_handler(job_id: str):
    """
    Common handler for stopping test jobs (performance or density).

    This helper function encapsulates the shared logic for stopping test jobs
    and mapping the outcome to appropriate HTTP status codes.

    Parameters
    ----------
    job_id : str
        Identifier of the test job to stop.

    Returns
    -------
    MessageResponse | JSONResponse
        A :class:`MessageResponse` instance (directly for success; wrapped
        in :class:`JSONResponse` for non-200 cases) describing the result
        of the stop attempt.
    """
    success, message = tests_manager.stop_job(job_id)
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


@router.get(
    "/tests/performance/status",
    operation_id="get_performance_statuses",
    response_model=List[schemas.PerformanceJobStatus],
)
def get_performance_statuses():
    """
    Return the status of all known performance test jobs.

    This endpoint provides a list of :class:`PerformanceJobStatus` objects,
    each describing the current state and metrics of a performance test job
    that has been started via the API. The response includes:

    * job identifier,
    * current state (RUNNING, COMPLETED, ERROR, ABORTED),
    * timing information (start time, elapsed time),
    * performance metrics (total FPS, per-stream FPS, total streams),
    * pipeline configuration details (streams per pipeline),
    * error messages (if any).

    The implementation delegates to the global :data:`tests_manager` instance,
    which aggregates and logs all known performance jobs. The returned list
    is automatically serialized to JSON by FastAPI using the Pydantic model
    definitions.

    Returns
    -------
    List[PerformanceJobStatus]
        A list of :class:`PerformanceJobStatus` objects, one for each
        performance test job currently tracked by the system.
    """
    return tests_manager.get_job_statuses_by_type(PerformanceJob)


@router.get(
    "/tests/performance/{job_id}/status",
    operation_id="get_performance_job_status",
    responses={
        200: {
            "description": "Successful Response",
            "model": schemas.PerformanceJobStatus,
        },
        404: {"description": "Job not found", "model": schemas.MessageResponse},
    },
)
def get_performance_job_status(job_id: str):
    """
    Return the detailed status of a specific performance test job.

    This endpoint focuses on a single performance test job and provides the
    complete :class:`PerformanceJobStatus` object for it.

    Parameters
    ----------
    job_id : str
        Identifier of the performance test job whose status should be
        retrieved.

    Returns
    -------
    PerformanceJobStatus | JSONResponse
        * On success (job exists): the full :class:`PerformanceJobStatus`
          instance describing the current state and metrics.
        * On failure (job unknown): a ``404`` :class:`JSONResponse` containing
          a :class:`MessageResponse` with a human-readable explanation.
    """
    return get_job_status_or_404(job_id, "Performance")


@router.get(
    "/tests/performance/{job_id}",
    operation_id="get_performance_job_summary",
    responses={
        200: {
            "description": "Successful Response",
            "model": schemas.PerformanceJobSummary,
        },
        404: {"description": "Job not found", "model": schemas.MessageResponse},
    },
)
def get_performance_job_summary(job_id: str):
    """
    Return a short summary of a specific performance test job.

    Parameters
    ----------
    job_id : str
        The identifier of the performance test job whose summary should
        be retrieved. This id is the value previously returned by the
        job creation endpoint.

    Returns
    -------
    PerformanceJobSummary | JSONResponse
        * On success (job exists): a :class:`PerformanceJobSummary` instance
          capturing the job id and original request parameters.
        * On failure (unknown id): a ``404`` :class:`JSONResponse` whose body
          is a :class:`MessageResponse` explaining that the job was not found.
    """
    summary = tests_manager.get_job_summary(job_id)
    if summary is None:
        return JSONResponse(
            content=schemas.MessageResponse(
                message=f"Job {job_id} not found"
            ).model_dump(),
            status_code=404,
        )
    return summary


@router.delete(
    "/tests/performance/{job_id}",
    operation_id="stop_performance_test_job",
    responses={
        200: {
            "description": "Successful Response",
            "model": schemas.MessageResponse,
        },
        404: {
            "description": "Performance test job not found",
            "model": schemas.MessageResponse,
        },
        409: {
            "description": "Performance test job not running",
            "model": schemas.MessageResponse,
        },
        500: {
            "description": "Unexpected error",
            "model": schemas.MessageResponse,
        },
    },
)
def stop_performance_test_job(job_id: str):
    """
    Stop a running performance test job.

    This endpoint attempts to terminate the active runner associated with
    the specified performance test job. The outcome of the stop request is mapped to
    explicit HTTP status codes for straightforward client handling:

    * ``200`` – Job successfully stopped.
    * ``404`` – Unknown job id or no active runner (never started / purged).
      Indicates the resource was not found.
    * ``409`` – Job exists but is not in a RUNNING state (COMPLETED, ABORTED,
      ERROR). Represents a conflicting state transition.
    * ``500`` – Unexpected internal failure while attempting to stop
      (e.g. thread interruption issue). Provides a diagnostic message.


    Parameters
    ----------
    job_id : str
            Identifier of the performance test job whose execution should be
            stopped.

    Returns
    -------
    MessageResponse | JSONResponse
            A :class:`MessageResponse` instance (directly for success; wrapped
            in :class:`JSONResponse` for non-200 cases) describing the result
            of the stop attempt.
    """
    return stop_test_job_handler(job_id)


@router.get(
    "/tests/density/status",
    operation_id="get_density_statuses",
    response_model=List[schemas.DensityJobStatus],
)
def get_density_statuses():
    """
    Return the status of all known density test jobs.

    This endpoint provides a list of :class:`DensityJobStatus` objects,
    each describing the current state and metrics of a density test job
    that has been started via the API. The response includes:

    * job identifier,
    * current state (RUNNING, COMPLETED, ERROR, ABORTED),
    * timing information (start time, elapsed time),
    * performance metrics (total FPS, per-stream FPS, total streams),
    * pipeline configuration details (streams per pipeline),
    * error messages (if any).

    The implementation delegates to the global :data:`tests_manager` instance,
    which aggregates and logs all known density jobs. The returned list
    is automatically serialized to JSON by FastAPI using the Pydantic model
    definitions.

    Returns
    -------
    List[DensityJobStatus]
        A list of :class:`DensityJobStatus` objects, one for each
        density test job currently tracked by the system.
    """
    return tests_manager.get_job_statuses_by_type(DensityJob)


@router.get(
    "/tests/density/{job_id}/status",
    operation_id="get_density_job_status",
    responses={
        200: {
            "description": "Successful Response",
            "model": schemas.DensityJobStatus,
        },
        404: {"description": "Job not found", "model": schemas.MessageResponse},
    },
)
def get_density_job_status(job_id: str):
    """
    Return the detailed status of a specific density test job.

    This endpoint focuses on a single density test job and provides the
    complete :class:`DensityJobStatus` object for it.

    Parameters
    ----------
    job_id : str
        Identifier of the density test job whose status should be
        retrieved.

    Returns
    -------
    DensityJobStatus | JSONResponse
        * On success (job exists): the full :class:`DensityJobStatus`
          instance describing the current state and metrics.
        * On failure (job unknown): a ``404`` :class:`JSONResponse` containing
          a :class:`MessageResponse` with a human-readable explanation.
    """
    return get_job_status_or_404(job_id, "Density")


@router.get(
    "/tests/density/{job_id}",
    operation_id="get_density_job_summary",
    responses={
        200: {
            "description": "Successful Response",
            "model": schemas.DensityJobSummary,
        },
        404: {"description": "Job not found", "model": schemas.MessageResponse},
    },
)
def get_density_job_summary(job_id: str):
    """
    Return a short summary of a specific density test job.

    Parameters
    ----------
    job_id : str
        The identifier of the density test job whose summary should
        be retrieved. This id is the value previously returned by the
        job creation endpoint.

    Returns
    -------
    DensityJobSummary | JSONResponse
        * On success (job exists): a :class:`DensityJobSummary` instance
          capturing the job id and original request parameters.
        * On failure (unknown id): a ``404`` :class:`JSONResponse` whose body
          is a :class:`MessageResponse` explaining that the job was not found.
    """
    summary = tests_manager.get_job_summary(job_id)
    if summary is None:
        return JSONResponse(
            content=schemas.MessageResponse(
                message=f"Job {job_id} not found"
            ).model_dump(),
            status_code=404,
        )
    return summary


@router.delete(
    "/tests/density/{job_id}",
    operation_id="stop_density_test_job",
    responses={
        200: {
            "description": "Successful Response",
            "model": schemas.MessageResponse,
        },
        404: {
            "description": "Density test job not found",
            "model": schemas.MessageResponse,
        },
        409: {
            "description": "Density test job not running",
            "model": schemas.MessageResponse,
        },
        500: {
            "description": "Unexpected error",
            "model": schemas.MessageResponse,
        },
    },
)
def stop_density_test_job(job_id: str):
    """
    Stop a running density test job.

    This endpoint attempts to terminate the active runner associated with
    the specified density test job. The outcome of the stop request is mapped to
    explicit HTTP status codes for straightforward client handling:

    * ``200`` – Job successfully stopped.
    * ``404`` – Unknown job id or no active runner (never started / purged).
      Indicates the resource was not found.
    * ``409`` – Job exists but is not in a RUNNING state (COMPLETED, ABORTED,
      ERROR). Represents a conflicting state transition.
    * ``500`` – Unexpected internal failure while attempting to stop
      (e.g. thread interruption issue). Provides a diagnostic message.


    Parameters
    ----------
    job_id : str
            Identifier of the density test job whose execution should be
            stopped.

    Returns
    -------
    MessageResponse | JSONResponse
            A :class:`MessageResponse` instance (directly for success; wrapped
            in :class:`JSONResponse` for non-200 cases) describing the result
            of the stop attempt.
    """
    return stop_test_job_handler(job_id)


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


@router.get(
    "/validation/status",
    operation_id="get_validation_statuses",
    response_model=List[schemas.ValidationJobStatus],
)
def get_validation_statuses():
    """
    Return the status of all known validation jobs.

    The response is a list of :class:`ValidationJobStatus` objects
    describing each job, including:

    * job identifier,
    * current state (RUNNING, COMPLETED, ERROR, ABORTED),
    * timing information (start / elapsed),
    * final validation result (``is_valid`` flag),
    * detailed error messages (if any).
    """
    return validation_manager.get_all_job_statuses()


@router.get(
    "/validation/{job_id}",
    operation_id="get_validation_job_summary",
    responses={
        200: {
            "description": "Successful Response",
            "model": schemas.ValidationJobSummary,
        },
        404: {
            "description": "Validation job not found",
            "model": schemas.MessageResponse,
        },
    },
)
def get_validation_job_summary(job_id: str):
    """
    Return a short summary of a specific validation job.

    Parameters
    ----------
    job_id:
        The identifier previously returned when the job was created.

    Returns
    -------
    ValidationJobSummary | JSONResponse
        * On success (job exists): a :class:`ValidationJobSummary`
          instance containing the job id and the original validation
          request.
        * On failure (job is unknown): a ``404`` :class:`JSONResponse`
          whose body is a :class:`MessageResponse`.
    """
    summary = validation_manager.get_job_summary(job_id)
    if summary is None:
        return JSONResponse(
            content=schemas.MessageResponse(
                message=f"Validation job {job_id} not found"
            ).model_dump(),
            status_code=404,
        )
    return summary


@router.get(
    "/validation/{job_id}/status",
    operation_id="get_validation_job_status",
    responses={
        200: {
            "description": "Successful Response",
            "model": schemas.ValidationJobStatus,
        },
        404: {
            "description": "Validation job not found",
            "model": schemas.MessageResponse,
        },
    },
)
def get_validation_job_status(job_id: str):
    """
    Return the detailed status of a specific validation job.

    Parameters
    ----------
    job_id:
        Identifier of the validation job whose status should be
        retrieved.

    Returns
    -------
    ValidationJobStatus | JSONResponse
        * On success (job exists): a :class:`ValidationJobStatus`
          instance describing the current state and outcome.
        * On failure (job is unknown): a ``404`` :class:`JSONResponse`
          with a :class:`MessageResponse` body.
    """
    status = validation_manager.get_job_status(job_id)
    if status is None:
        return JSONResponse(
            content=schemas.MessageResponse(
                message=f"Validation job {job_id} not found"
            ).model_dump(),
            status_code=404,
        )
    return status

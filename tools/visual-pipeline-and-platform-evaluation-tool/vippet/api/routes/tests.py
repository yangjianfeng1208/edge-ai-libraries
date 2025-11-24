from fastapi import APIRouter

import api.api_schemas as schemas
from managers.tests_manager import get_tests_manager

router = APIRouter()
test_manager = get_tests_manager()


@router.post(
    "/performance",
    operation_id="run_performance_test",
    status_code=202,
    response_model=schemas.TestJobResponse,
)
def run_performance_test(body: schemas.PerformanceTestSpec):
    """Run a performance test."""
    job_id = test_manager.test_performance(body)
    return schemas.TestJobResponse(job_id=job_id)


@router.post(
    "/density",
    operation_id="run_density_test",
    status_code=202,
    response_model=schemas.TestJobResponse,
)
def run_density_test(body: schemas.DensityTestSpec):
    """Run a density test."""
    job_id = test_manager.test_density(body)
    return schemas.TestJobResponse(job_id=job_id)

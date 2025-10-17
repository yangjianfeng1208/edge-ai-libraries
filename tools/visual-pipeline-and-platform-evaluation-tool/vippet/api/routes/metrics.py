import api.api_schemas as schemas
from typing import List
from fastapi import APIRouter

router = APIRouter()


@router.get("", response_model=List[schemas.MetricSample])
def get_metrics():
    return []

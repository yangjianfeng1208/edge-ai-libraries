import logging
import yaml
import os

from typing import List, Union, Dict, Any, Optional
from fastapi import Body, FastAPI
from pydantic import BaseModel

from gstpipeline import PipelineLoader
from optimize import PipelineOptimizer
from explore import GstInspector

from pipelines.pipeline_page import download_file
from utils import prepare_video_and_constants
from device import DeviceDiscovery
from models import SupportedModelsManager


TEMP_DIR = "/tmp/"

class Source(BaseModel):
    type: str
    uri: str

class PipelineRunBody(BaseModel):
    async_: Optional[bool] = Body(default=False, alias="async")
    source: Source
    parameters: Dict[str, Any]
    tags: Optional[Union[List[str], Dict[str, Any]]] = {}


with open("api/vippet.yaml") as f:
    openapi_schema = yaml.safe_load(f)

app = FastAPI()
app.openapi = lambda: openapi_schema

gst_inspector = GstInspector()

@app.get("/")
def read_root():
    return {"Hello": "World"}

@app.get("/devices")
def read_devices():
    device_discovery = DeviceDiscovery()
    device_list = device_discovery.list_devices()
    return [device.__dict__ for device in device_list]

@app.get("/models")
def read_models():
    models = SupportedModelsManager().get_all_available_models()
    return [
        {
            "name": m.name,
            "display_name": m.display_name,
            "category": m.model_type,
            "precision": m.display_name.split(" ")[-1].strip("()")
        }
        for m in models
    ]

@app.get("/pipelines")
def read_pipelines():
    pipeline_infos = []
    for pipeline in PipelineLoader.list():
        config = PipelineLoader.config(pipeline)
        pipeline_infos.append({
            "name": config.get("name", "Unnamed Pipeline"),
            "version": config.get("version", "0.0.1"),
            "description": config.get("definition", ""),
            "type": config.get("type", "GStreamer"),
        })
    return pipeline_infos


@app.get("/pipelines/{pipeline_id}")
def get_pipeline_info(pipeline_id: str):
    if not pipeline_id in PipelineLoader.list():
        return {"error": "Pipeline not found"}

    return PipelineLoader.config(pipeline_id)


@app.post("/pipelines/{name}/{version}/run")
def run_pipeline(
    name: str,
    version: str,
    body: PipelineRunBody
):
    dir = "smartnvr"
    gst_pipeline, config = PipelineLoader.load(dir)

    # Download the pipeline recording files
    download_file(
        config["recording"]["url"],
        config["recording"]["filename"],
    )

    body.parameters["input_video_player"] = os.path.join(TEMP_DIR, config["recording"]["filename"])

    try:
        video_output_path, constants, param_grid = prepare_video_and_constants(**body.parameters)
    except ValueError as e:
        return {"error": str(e)}

    logging.info(f"Constants: {constants}")
    logging.info(f"param_grid: {param_grid}")

    recording_channels = 0
    inferencing_channels = body.parameters["inferencing_channels"]

    if recording_channels + inferencing_channels == 0:
        return {"error": "At least one channel must be enabled"}

    optimizer = PipelineOptimizer(
        pipeline=gst_pipeline,
        constants=constants,
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

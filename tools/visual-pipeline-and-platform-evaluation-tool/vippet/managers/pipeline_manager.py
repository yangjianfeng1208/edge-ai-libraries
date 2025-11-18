import logging
import sys
from typing import Optional

from pipelines.loader import PipelineLoader
from utils import make_tee_names_unique
from graph import Graph
from api.api_schemas import (
    PipelineType,
    Pipeline,
    PipelineDefinition,
    PipelineRunSpec,
    PipelineGraph,
)

logger = logging.getLogger("pipeline_manager")

# Singleton instance for PipelineManager
_pipeline_manager_instance: Optional["PipelineManager"] = None


def get_pipeline_manager() -> "PipelineManager":
    """
    Returns the singleton instance of PipelineManager.
    If it cannot be created, logs an error and exits the application.
    """
    global _pipeline_manager_instance
    if _pipeline_manager_instance is None:
        try:
            _pipeline_manager_instance = PipelineManager()
        except Exception as e:
            logger.error(f"Failed to initialize PipelineManager: {e}")
            sys.exit(1)
    return _pipeline_manager_instance


class PipelineManager:
    def __init__(self):
        self.logger = logging.getLogger("PipelineManager")
        self.pipelines = self.load_predefined_pipelines()

    def add_pipeline(self, new_pipeline: PipelineDefinition):
        if self.pipeline_exists(new_pipeline.name, new_pipeline.version):
            raise ValueError(
                f"Pipeline with name '{new_pipeline.name}' and version '{new_pipeline.version}' already exists."
            )

        pipeline_graph = Graph.from_pipeline_description(
            new_pipeline.pipeline_description
        ).to_dict()

        pipeline = Pipeline(
            name=new_pipeline.name,
            version=new_pipeline.version,
            description=new_pipeline.description,
            type=new_pipeline.type,
            pipeline_graph=PipelineGraph.model_validate(pipeline_graph),
            parameters=new_pipeline.parameters,
        )

        self.pipelines.append(pipeline)
        self.logger.debug(f"Pipeline added: {pipeline}")

    def get_pipelines(self) -> list[Pipeline]:
        return self.pipelines

    def get_pipeline_by_name_and_version(self, name: str, version: str) -> Pipeline:
        pipeline = self._find_pipeline(name, version)
        if pipeline is not None:
            return pipeline
        raise ValueError(
            f"Pipeline with name '{name}' and version '{version}' not found."
        )

    def pipeline_exists(self, name: str, version: str) -> bool:
        return self._find_pipeline(name, version) is not None

    def _find_pipeline(self, name: str, version: str) -> Pipeline | None:
        for pipeline in self.pipelines:
            if pipeline.name == name and pipeline.version == version:
                return pipeline
        return None

    def load_predefined_pipelines(self):
        predefined_pipelines = []
        for pipeline_name in PipelineLoader.list():
            config = PipelineLoader.config(pipeline_name)

            pipeline_description = config.get("pipeline_description", "")
            pipeline_graph = Graph.from_pipeline_description(
                pipeline_description
            ).to_dict()

            predefined_pipelines.append(
                Pipeline(
                    name="predefined_pipelines",
                    version=config.get("name", "UnnamedPipeline"),
                    description=config.get("display_name", "Unnamed Pipeline"),
                    type=PipelineType.GSTREAMER,
                    pipeline_graph=PipelineGraph.model_validate(pipeline_graph),
                    parameters=None,
                )
            )
        self.logger.debug("Loaded predefined pipelines: %s", predefined_pipelines)
        return predefined_pipelines

    def build_pipeline_command(self, pipeline_run_specs: list[PipelineRunSpec]) -> str:
        """
        Build a complete GStreamer pipeline command from run specifications.

        Args:
            pipeline_run_specs: List of PipelineRunSpec defining pipelines and streams.

        Returns:
            str: Complete GStreamer pipeline command string.

        Raises:
            ValueError: If any pipeline in specs is not found.
        """
        pipeline_parts = []

        for pipeline_index, run_spec in enumerate(pipeline_run_specs):
            # Retrieve the pipeline definition
            pipeline = self.get_pipeline_by_name_and_version(
                run_spec.name, run_spec.version
            )

            # Extract the pipeline description string
            base_pipeline_str = Graph.from_dict(
                pipeline.pipeline_graph.model_dump()
            ).to_pipeline_description()

            # Create one pipeline instance per stream with unique tee names
            for stream_index in range(run_spec.streams):
                unique_pipeline_str = make_tee_names_unique(
                    base_pipeline_str, pipeline_index, stream_index
                )
                pipeline_parts.append(unique_pipeline_str)

        return " ".join(pipeline_parts)

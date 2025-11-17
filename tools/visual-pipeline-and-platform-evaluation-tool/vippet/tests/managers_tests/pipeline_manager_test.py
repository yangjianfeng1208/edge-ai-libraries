import unittest

from managers.pipeline_manager import PipelineManager
from api.api_schemas import PipelineType, PipelineDefinition


class TestPipelineManager(unittest.TestCase):
    def test_add_pipeline_valid(self):
        manager = PipelineManager()
        manager.pipelines = []  # Reset pipelines for isolated test
        initial_count = len(manager.get_pipelines())

        new_pipeline = PipelineDefinition(
            name="user-defined-pipelines",
            version="test-pipeline",
            description="A test pipeline",
            type=PipelineType.GSTREAMER,
            pipeline_description="filesrc location=/tmp/dummy-video.mp4 ! decodebin3 ! autovideosink",
            parameters=None,
        )

        manager.add_pipeline(new_pipeline)
        pipelines = manager.get_pipelines()
        self.assertEqual(len(pipelines), initial_count + 1)

        # Verify the added pipeline
        added_pipeline = manager.get_pipeline_by_name_and_version(
            "user-defined-pipelines", "test-pipeline"
        )
        self.assertEqual(added_pipeline.name, "user-defined-pipelines")
        self.assertEqual(added_pipeline.version, "test-pipeline")

    def test_add_pipeline_duplicate(self):
        manager = PipelineManager()
        manager.pipelines = []  # Reset pipelines for isolated test

        new_pipeline = PipelineDefinition(
            name="user-defined-pipelines",
            version="test-pipeline",
            description="A test pipeline",
            type=PipelineType.GSTREAMER,
            pipeline_description="filesrc location=/tmp/dummy-video.mp4 ! decodebin3 ! autovideosink",
            parameters=None,
        )

        manager.add_pipeline(new_pipeline)

        # Attempt to add the same pipeline again should raise ValueError
        with self.assertRaises(ValueError) as context:
            manager.add_pipeline(new_pipeline)

        self.assertIn(
            "Pipeline with name 'user-defined-pipelines' and version 'test-pipeline' already exists.",
            str(context.exception),
        )

    def test_get_pipeline_by_name_and_version_not_found(self):
        manager = PipelineManager()

        with self.assertRaises(ValueError) as context:
            manager.get_pipeline_by_name_and_version(
                "user-defined-pipelines", "nonexistent-pipeline"
            )

        self.assertIn(
            "Pipeline with name 'user-defined-pipelines' and version 'nonexistent-pipeline' not found.",
            str(context.exception),
        )

    def test_load_predefined_pipelines(self):
        manager = PipelineManager()
        pipelines = manager.get_pipelines()
        self.assertIsInstance(pipelines, list)
        self.assertGreaterEqual(len(pipelines), 3)

        # Define expected pipelines (name, version, description)
        expected = [
            (
                "predefined_pipelines",
                "simplevs",
                "Simple Video Structurization (D-T-C)",
            ),
            (
                "predefined_pipelines",
                "smartnvr-analytics",
                "Smart Network Video Recorder (NVR) Proxy Pipeline - Analytics Branch",
            ),
            (
                "predefined_pipelines",
                "smartnvr-mediaonly",
                "Smart Network Video Recorder (NVR) Proxy Pipeline - Media Only Branch",
            ),
        ]

        # Check that each expected pipeline is present in the loaded pipelines
        for exp_name, exp_version, exp_desc in expected:
            found = [
                p
                for p in pipelines
                if p.name == exp_name
                and p.version == exp_version
                and p.description == exp_desc
            ]
            self.assertTrue(found, f"Pipeline {exp_name} {exp_version} not found")
            self.assertIsNotNone(found[0].pipeline_graph)

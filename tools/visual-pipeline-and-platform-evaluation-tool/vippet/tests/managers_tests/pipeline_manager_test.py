import unittest

from managers.pipeline_manager import PipelineManager
from api.api_schemas import PipelineType, PipelineDefinition, PipelineRunSpec


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

    def test_build_pipeline_command_single_pipeline_single_stream(self):
        manager = PipelineManager()

        # Add a test pipeline
        test_pipeline = PipelineDefinition(
            name="test-pipelines",
            version="test-single",
            description="Test pipeline for single stream",
            type=PipelineType.GSTREAMER,
            pipeline_description="fakesrc ! fakesink",
            parameters=None,
        )
        manager.add_pipeline(test_pipeline)

        # Build command with one pipeline and one stream
        pipeline_run_specs = [
            PipelineRunSpec(name="test-pipelines", version="test-single", streams=1)
        ]

        command = manager.build_pipeline_command(pipeline_run_specs)

        # Verify command is not empty and contains pipeline elements
        self.assertIsInstance(command, str)
        self.assertGreater(len(command), 0)
        self.assertIn("fakesrc", command)
        self.assertIn("fakesink", command)

    def test_build_pipeline_command_single_pipeline_multiple_streams(self):
        manager = PipelineManager()

        # Add a test pipeline
        test_pipeline = PipelineDefinition(
            name="test-pipelines",
            version="test-multi",
            description="Test pipeline for multiple streams",
            type=PipelineType.GSTREAMER,
            pipeline_description="videotestsrc ! tee name=t ! queue ! fakesink t. ! queue ! fakesink",
            parameters=None,
        )
        manager.add_pipeline(test_pipeline)

        # Build command with one pipeline and 3 streams
        pipeline_run_specs = [
            PipelineRunSpec(name="test-pipelines", version="test-multi", streams=3)
        ]

        command = manager.build_pipeline_command(pipeline_run_specs)

        # Verify command contains multiple instances
        self.assertIsInstance(command, str)
        self.assertGreater(len(command), 0)
        # Should have 3 instances of videotestsrc (one per stream)
        self.assertEqual(command.count("videotestsrc"), 3)

    def test_build_pipeline_command_multiple_pipelines(self):
        manager = PipelineManager()

        # Add two test pipelines
        pipeline1 = PipelineDefinition(
            name="test-pipelines",
            version="pipeline1",
            description="First test pipeline",
            type=PipelineType.GSTREAMER,
            pipeline_description="fakesrc name=source1 ! fakesink",
            parameters=None,
        )
        pipeline2 = PipelineDefinition(
            name="test-pipelines",
            version="pipeline2",
            description="Second test pipeline",
            type=PipelineType.GSTREAMER,
            pipeline_description="videotestsrc name=source2 ! fakesink",
            parameters=None,
        )
        manager.add_pipeline(pipeline1)
        manager.add_pipeline(pipeline2)

        # Build command with two pipelines with different stream counts
        pipeline_run_specs = [
            PipelineRunSpec(name="test-pipelines", version="pipeline1", streams=2),
            PipelineRunSpec(name="test-pipelines", version="pipeline2", streams=3),
        ]

        command = manager.build_pipeline_command(pipeline_run_specs)

        # Verify both pipeline types are present
        self.assertIsInstance(command, str)
        self.assertGreater(len(command), 0)
        # Should have 2 instances of fakesrc and 3 instances of videotestsrc
        self.assertEqual(command.count("fakesrc"), 2)
        self.assertEqual(command.count("videotestsrc"), 3)

    def test_build_pipeline_command_nonexistent_pipeline_raises_error(self):
        manager = PipelineManager()

        # Try to build command with pipeline that doesn't exist
        pipeline_run_specs = [
            PipelineRunSpec(name="nonexistent", version="missing", streams=1)
        ]

        with self.assertRaises(ValueError) as context:
            manager.build_pipeline_command(pipeline_run_specs)

        self.assertIn(
            "Pipeline with name 'nonexistent' and version 'missing' not found",
            str(context.exception),
        )

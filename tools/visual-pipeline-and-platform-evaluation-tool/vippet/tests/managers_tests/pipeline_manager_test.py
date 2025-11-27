import unittest

from managers.pipeline_manager import PipelineManager
from api.api_schemas import (
    Node,
    Edge,
    PipelineType,
    PipelineSource,
    PipelineGraph,
    PipelineParameters,
    PipelineDefinition,
    PipelinePerformanceSpec,
)


class TestPipelineManager(unittest.TestCase):
    def test_add_pipeline_valid(self):
        manager = PipelineManager()
        manager.pipelines = []  # Reset pipelines for isolated test
        initial_count = len(manager.get_pipelines())

        new_pipeline = PipelineDefinition(
            name="user-defined-pipelines",
            version=1,
            description="A test pipeline",
            source=PipelineSource.USER_CREATED,
            type=PipelineType.GSTREAMER,
            pipeline_description="filesrc location=/tmp/dummy-video.mp4 ! decodebin3 ! autovideosink",
            parameters=None,
        )

        added_pipeline = manager.add_pipeline(new_pipeline)
        pipelines = manager.get_pipelines()
        self.assertEqual(len(pipelines), initial_count + 1)

        # Verify the added pipeline has an ID and correct attributes
        self.assertIsNotNone(added_pipeline.id)
        self.assertTrue(added_pipeline.id.startswith("pipeline-"))
        self.assertEqual(added_pipeline.name, "user-defined-pipelines")
        self.assertEqual(added_pipeline.version, 1)

        # Verify we can retrieve it by ID
        retrieved = manager.get_pipeline_by_id(added_pipeline.id)
        self.assertEqual(retrieved.name, "user-defined-pipelines")
        self.assertEqual(retrieved.version, 1)

    def test_add_pipeline_duplicate(self):
        manager = PipelineManager()
        manager.pipelines = []  # Reset pipelines for isolated test

        new_pipeline = PipelineDefinition(
            name="user-defined-pipelines",
            version=1,
            description="A test pipeline",
            source=PipelineSource.USER_CREATED,
            type=PipelineType.GSTREAMER,
            pipeline_description="filesrc location=/tmp/dummy-video.mp4 ! decodebin3 ! autovideosink",
            parameters=None,
        )

        manager.add_pipeline(new_pipeline)

        # Attempt to add the same pipeline again should raise ValueError
        with self.assertRaises(ValueError) as context:
            manager.add_pipeline(new_pipeline)

        self.assertIn(
            "Pipeline with name 'user-defined-pipelines' and version '1' already exists.",
            str(context.exception),
        )

    def test_get_pipeline_by_id_not_found(self):
        manager = PipelineManager()

        with self.assertRaises(ValueError) as context:
            manager.get_pipeline_by_id("nonexistent-pipeline-id")

        self.assertIn(
            "Pipeline with id 'nonexistent-pipeline-id' not found.",
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
                "Simple Video Structurization (D-T-C)",
                1,
                "Test Pipeline Description",
            ),
            (
                "Smart Network Video Recorder (NVR) Proxy Pipeline - Analytics Branch",
                1,
                "Test Pipeline Description",
            ),
            (
                "Smart Network Video Recorder (NVR) Proxy Pipeline - Media Only Branch",
                1,
                "Test Pipeline Description",
            ),
        ]

        # Check that each expected pipeline is present in the loaded pipelines
        for exp_name, exp_version, exp_desc in expected:
            found = [
                p for p in pipelines if p.name == exp_name and p.version == exp_version
            ]
            self.assertTrue(found, f"Pipeline {exp_name} {exp_version} not found")
            self.assertIsNotNone(found[0].pipeline_graph)

    def test_build_pipeline_command_single_pipeline_single_stream(self):
        manager = PipelineManager()

        # Add a test pipeline
        test_pipeline = PipelineDefinition(
            name="test-pipelines",
            version=1,
            description="Test pipeline for single stream",
            source=PipelineSource.USER_CREATED,
            type=PipelineType.GSTREAMER,
            pipeline_description="fakesrc ! fakesink",
            parameters=None,
        )
        added = manager.add_pipeline(test_pipeline)

        # Build command with one pipeline and one stream using the pipeline ID
        pipeline_performance_specs = [PipelinePerformanceSpec(id=added.id, streams=1)]

        command = manager.build_pipeline_command(pipeline_performance_specs)

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
            version=1,
            description="Test pipeline for multiple streams",
            source=PipelineSource.USER_CREATED,
            type=PipelineType.GSTREAMER,
            pipeline_description="videotestsrc ! tee name=t ! queue ! fakesink t. ! queue ! fakesink",
            parameters=None,
        )
        added = manager.add_pipeline(test_pipeline)

        # Build command with one pipeline and 3 streams using the pipeline ID
        pipeline_performance_specs = [PipelinePerformanceSpec(id=added.id, streams=3)]

        command = manager.build_pipeline_command(pipeline_performance_specs)

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
            version=1,
            description="First test pipeline",
            source=PipelineSource.USER_CREATED,
            type=PipelineType.GSTREAMER,
            pipeline_description="fakesrc name=source1 ! fakesink",
            parameters=None,
        )
        pipeline2 = PipelineDefinition(
            name="test-pipelines",
            version=2,
            description="Second test pipeline",
            source=PipelineSource.USER_CREATED,
            type=PipelineType.GSTREAMER,
            pipeline_description="videotestsrc name=source2 ! fakesink",
            parameters=None,
        )
        added1 = manager.add_pipeline(pipeline1)
        added2 = manager.add_pipeline(pipeline2)

        # Build command with two pipelines with different stream counts using IDs
        pipeline_performance_specs = [
            PipelinePerformanceSpec(id=added1.id, streams=2),
            PipelinePerformanceSpec(id=added2.id, streams=3),
        ]

        command = manager.build_pipeline_command(pipeline_performance_specs)

        # Verify both pipeline types are present
        self.assertIsInstance(command, str)
        self.assertGreater(len(command), 0)
        # Should have 2 instances of fakesrc and 3 instances of videotestsrc
        self.assertEqual(command.count("fakesrc"), 2)
        self.assertEqual(command.count("videotestsrc"), 3)

    def test_build_pipeline_command_nonexistent_pipeline_raises_error(self):
        manager = PipelineManager()

        # Try to build command with pipeline ID that doesn't exist
        pipeline_performance_specs = [
            PipelinePerformanceSpec(id="nonexistent-pipeline-id", streams=1)
        ]

        with self.assertRaises(ValueError) as context:
            manager.build_pipeline_command(pipeline_performance_specs)

        self.assertIn(
            "Pipeline with id 'nonexistent-pipeline-id' not found",
            str(context.exception),
        )

    def test_update_pipeline_description_and_name(self):
        manager = PipelineManager()
        manager.pipelines = []

        new_pipeline = PipelineDefinition(
            name="original-name",
            version=1,
            description="Original description",
            source=PipelineSource.USER_CREATED,
            type=PipelineType.GSTREAMER,
            pipeline_description="fakesrc ! fakesink",
            parameters=None,
        )

        added = manager.add_pipeline(new_pipeline)

        updated = manager.update_pipeline(
            pipeline_id=added.id,
            name="updated-name",
            description="Updated description",
        )

        self.assertEqual(updated.id, added.id)
        self.assertEqual(updated.name, "updated-name")
        self.assertEqual(updated.description, "Updated description")

        # Ensure the change is reflected in manager state
        retrieved = manager.get_pipeline_by_id(added.id)
        self.assertEqual(retrieved.name, "updated-name")
        self.assertEqual(retrieved.description, "Updated description")

    def test_update_pipeline_graph_and_parameters(self):
        manager = PipelineManager()
        manager.pipelines = []

        new_pipeline = PipelineDefinition(
            name="test-pipeline",
            version=1,
            description="Pipeline to be updated",
            source=PipelineSource.USER_CREATED,
            type=PipelineType.GSTREAMER,
            pipeline_description="fakesrc ! fakesink",
            parameters=None,
        )

        added = manager.add_pipeline(new_pipeline)

        updated_graph = PipelineGraph(
            nodes=[
                Node(id="0", type="videotestsrc", data={}),
                Node(id="1", type="fakesink", data={}),
            ],
            edges=[Edge(id="0", source="0", target="1")],
        )

        updated_params = PipelineParameters(default={"key": "value"})

        updated = manager.update_pipeline(
            pipeline_id=added.id,
            pipeline_graph=updated_graph,
            parameters=updated_params,
        )

        self.assertEqual(updated.id, added.id)
        self.assertEqual(updated.pipeline_graph, updated_graph)
        self.assertEqual(updated.parameters, updated_params)

    def test_update_pipeline_not_found_raises(self):
        manager = PipelineManager()
        manager.pipelines = []

        with self.assertRaises(ValueError) as context:
            manager.update_pipeline(pipeline_id="nonexistent", name="new-name")

        self.assertIn(
            "Pipeline with id 'nonexistent' not found.", str(context.exception)
        )

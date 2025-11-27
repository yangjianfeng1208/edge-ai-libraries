import unittest

from videos import OUTPUT_VIDEO_DIR
from managers.pipeline_manager import PipelineManager
from api.api_schemas import (
    EncoderDeviceConfig,
    PipelineType,
    PipelineSource,
    PipelineDefinition,
    PipelinePerformanceSpec,
    VideoOutputConfig,
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
        video_config = VideoOutputConfig(enabled=False)

        command, output_paths = manager.build_pipeline_command(
            pipeline_performance_specs, video_config
        )

        # Verify command is not empty and contains pipeline elements
        self.assertIsInstance(command, str)
        self.assertIsInstance(output_paths, dict)
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
        video_config = VideoOutputConfig(enabled=False)

        command, output_paths = manager.build_pipeline_command(
            pipeline_performance_specs, video_config
        )

        # Verify command contains multiple instances
        self.assertIsInstance(command, str)
        self.assertIsInstance(output_paths, dict)
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
        video_config = VideoOutputConfig(enabled=False)

        command, output_paths = manager.build_pipeline_command(
            pipeline_performance_specs, video_config
        )

        # Verify both pipeline types are present
        self.assertIsInstance(command, str)
        self.assertIsInstance(output_paths, dict)
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
        video_config = VideoOutputConfig(enabled=False)

        with self.assertRaises(ValueError) as context:
            manager.build_pipeline_command(pipeline_performance_specs, video_config)

        self.assertIn(
            "Pipeline with id 'nonexistent-pipeline-id' not found",
            str(context.exception),
        )

    def test_build_pipeline_command_with_video_output_enabled(self):
        """Test building pipeline command with video output enabled."""
        manager = PipelineManager()
        manager.pipelines = []

        # Add a test pipeline
        new_pipeline = PipelineDefinition(
            name="test-video-output",
            version=1,
            description="Pipeline for testing video output",
            source=PipelineSource.USER_CREATED,
            type=PipelineType.GSTREAMER,
            pipeline_description="videotestsrc ! fakesink",
            parameters=None,
        )
        added = manager.add_pipeline(new_pipeline)

        pipeline_performance_specs = [PipelinePerformanceSpec(id=added.id, streams=1)]
        video_config = VideoOutputConfig(
            enabled=True,
            encoder_device=EncoderDeviceConfig(device_name="CPU", gpu_id=None),
        )

        command, output_paths = manager.build_pipeline_command(
            pipeline_performance_specs, video_config
        )

        # Verify video output is configured
        self.assertIsInstance(command, str)
        self.assertIsInstance(output_paths, dict)
        self.assertIn(added.id, output_paths)
        self.assertGreater(len(output_paths[added.id]), 0)

        # Verify output directory is in the command
        self.assertIn(OUTPUT_VIDEO_DIR, command)

        # Verify fakesink is replaced with encoder pipeline
        self.assertNotIn("fakesink", command)
        self.assertIn("filesink", command)

    def test_build_pipeline_command_with_gpu_encoder(self):
        """Test building pipeline command with GPU encoder."""
        manager = PipelineManager()
        manager.pipelines = []

        new_pipeline = PipelineDefinition(
            name="test-gpu-encoder",
            version=1,
            description="Pipeline with GPU encoder",
            source=PipelineSource.USER_CREATED,
            type=PipelineType.GSTREAMER,
            pipeline_description="videotestsrc ! fakesink",
            parameters=None,
        )
        added = manager.add_pipeline(new_pipeline)

        pipeline_performance_specs = [PipelinePerformanceSpec(id=added.id, streams=2)]
        video_config = VideoOutputConfig(
            enabled=True,
            encoder_device=EncoderDeviceConfig(device_name="GPU", gpu_id=0),
        )

        command, output_paths = manager.build_pipeline_command(
            pipeline_performance_specs, video_config
        )

        # Verify output paths for all streams
        self.assertIn(added.id, output_paths)
        # Should have only 1 output path (first stream)
        self.assertEqual(len(output_paths[added.id]), 1)
        # Verify output directory is in the command
        self.assertIn(OUTPUT_VIDEO_DIR, command)

    def test_build_pipeline_command_video_output_multiple_pipelines(self):
        """Test video output with multiple pipelines - only first stream gets output."""
        manager = PipelineManager()
        manager.pipelines = []

        pipeline1 = PipelineDefinition(
            name="pipeline-1",
            version=1,
            description="First pipeline",
            source=PipelineSource.USER_CREATED,
            type=PipelineType.GSTREAMER,
            pipeline_description="videotestsrc ! fakesink",
            parameters=None,
        )
        pipeline2 = PipelineDefinition(
            name="pipeline-2",
            version=1,
            description="Second pipeline",
            source=PipelineSource.USER_CREATED,
            type=PipelineType.GSTREAMER,
            pipeline_description="videotestsrc ! fakesink",
            parameters=None,
        )

        added1 = manager.add_pipeline(pipeline1)
        added2 = manager.add_pipeline(pipeline2)

        pipeline_performance_specs = [
            PipelinePerformanceSpec(id=added1.id, streams=2),
            PipelinePerformanceSpec(id=added2.id, streams=3),
        ]
        video_config = VideoOutputConfig(
            enabled=True,
            encoder_device=EncoderDeviceConfig(device_name="CPU", gpu_id=None),
        )

        command, output_paths = manager.build_pipeline_command(
            pipeline_performance_specs, video_config
        )

        # Verify video output paths exist for both pipelines
        self.assertIn(added1.id, output_paths)
        self.assertIn(added2.id, output_paths)

        # Each pipeline should have only 1 output path (first stream)
        self.assertEqual(len(output_paths[added1.id]), 1)
        self.assertEqual(len(output_paths[added2.id]), 1)

        # Verify output directory is in the command
        self.assertIn(OUTPUT_VIDEO_DIR, command)

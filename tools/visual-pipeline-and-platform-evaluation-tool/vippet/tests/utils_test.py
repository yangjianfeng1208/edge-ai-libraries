import os
import shutil
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import utils
from utils import (
    prepare_video_and_constants,
    is_yolov10_model,
)


class TestUtils(unittest.TestCase):
    def setUp(self):
        # Create a temporary directory for models and output
        self.temp_dir = tempfile.mkdtemp()
        self.models_path = os.path.join(self.temp_dir, "models")
        os.makedirs(self.models_path, exist_ok=True)
        self.input_video = os.path.join(self.temp_dir, "input.mp4")
        with open(self.input_video, "w") as f:
            f.write("dummy video content")

    def tearDown(self):
        shutil.rmtree(self.temp_dir)

    @patch("utils.discover_video_codec")
    @patch("utils.os.path.exists")
    @patch("utils.os.remove")
    @patch("utils.get_model_path_and_proc")
    def test_prepare_video_and_constants_output_file_removed(
        self,
        mock_get_model_path_and_proc,
        mock_remove,
        mock_exists,
        mock_discover_video_codec,
    ):
        mock_exists.return_value = True
        mock_discover_video_codec.return_value = "h264"
        mock_get_model_path_and_proc.side_effect = [
            ("/fake/path/detection.xml", "/fake/path/detection.proc"),
            ("/fake/path/classification.xml", "/fake/path/classification.proc"),
        ]
        output_path, constants, param_grid = prepare_video_and_constants(
            **{
                "input_video_player": self.input_video,
                "object_detection_model": "SSDLite MobileNet V2 (INT8)",
                "object_detection_device": "CPU",
                "object_detection_batch_size": 1,
                "object_detection_nireq": 1,
                "object_detection_inference_interval": 1,
                "object_classification_model": "ResNet-50 TF (INT8)",
                "object_classification_device": "CPU",
                "object_classification_batch_size": 1,
                "object_classification_nireq": 1,
                "object_classification_inference_interval": 1,
                "object_classification_reclassify_interval": 1,
            }
        )
        mock_remove.assert_called_once()
        self.assertTrue(output_path.endswith(".mp4"))
        self.assertIn("VIDEO_PATH", constants)
        self.assertIn("VIDEO_CODEC", constants)
        self.assertIn("VIDEO_OUTPUT_PATH", constants)
        self.assertIn("object_detection_device", param_grid)
        self.assertIn("object_classification_device", param_grid)

    @patch("utils.discover_video_codec")
    def test_prepare_video_and_constants_unknown_codec(self, mock_discover_video_codec):
        mock_discover_video_codec.return_value = "unknown"
        with self.assertRaises(ValueError) as context:
            prepare_video_and_constants(
                **{
                    "input_video_player": self.input_video,
                    "object_detection_model": "SSDLite MobileNet V2 (INT8)",
                    "object_detection_device": "CPU",
                    "object_detection_batch_size": 1,
                    "object_detection_nireq": 1,
                    "object_detection_inference_interval": 1,
                    "object_classification_model": "ResNet-50 TF (INT8)",
                    "object_classification_device": "CPU",
                    "object_classification_batch_size": 1,
                    "object_classification_nireq": 1,
                    "object_classification_inference_interval": 1,
                    "object_classification_reclassify_interval": 1,
                }
            )
        self.assertIn(
            "Could not detect the video codec of the input file. Please provide a valid video file.",
            str(context.exception),
        )

    @patch("utils.discover_video_codec")
    def test_prepare_video_and_constants_not_supported_codec(
        self, mock_discover_video_codec
    ):
        mock_discover_video_codec.return_value = (
            "av1"  # Assuming 'av1' is not in the supported list
        )
        with self.assertRaises(ValueError) as context:
            prepare_video_and_constants(
                **{
                    "input_video_player": self.input_video,
                    "object_detection_model": "SSDLite MobileNet V2 (INT8)",
                    "object_detection_device": "CPU",
                    "object_detection_batch_size": 1,
                    "object_detection_nireq": 1,
                    "object_detection_inference_interval": 1,
                    "object_classification_model": "ResNet-50 TF (INT8)",
                    "object_classification_device": "CPU",
                    "object_classification_batch_size": 1,
                    "object_classification_nireq": 1,
                    "object_classification_inference_interval": 1,
                    "object_classification_reclassify_interval": 1,
                    "video_codec": "unsupported_codec",
                }
            )
        self.assertIn(
            f"Input video codec '{mock_discover_video_codec.return_value}' is not supported.",
            str(context.exception),
        )

    @patch("cv2.VideoCapture")
    def test_discover_video_codec(self, mock_videocap):
        mock_cap = MagicMock()
        mock_cap.isOpened.return_value = True

        # Mock VideoCapture to simulate a video file with H.264 codec
        # 'avc' as fourcc: ord('a') | ord('v')<<8 | ord('c')<<16 | ord(' ')<<24
        fourcc = (ord("a")) | (ord("v") << 8) | (ord("c") << 16) | (ord(" ") << 24)
        mock_cap.get.return_value = fourcc
        mock_videocap.return_value = mock_cap
        video_codec = utils.discover_video_codec("dummy.mp4")
        self.assertEqual(video_codec, "h264")

        # Mock VideoCapture to simulate a video file with H.265 codec
        # 'hevc' as fourcc: ord('h') | ord('e')<<8 | ord('v')<<16 | ord('c')<<24
        fourcc = (ord("h")) | (ord("e") << 8) | (ord("v") << 16) | (ord("c") << 24)
        mock_cap.get.return_value = fourcc
        mock_videocap.return_value = mock_cap
        video_codec = utils.discover_video_codec("dummy.mp4")
        self.assertEqual(video_codec, "h265")

        # Mock VideoCapture to simulate a video file with AV1 codec
        # 'av01' as fourcc: ord('a') | ord('v')<<8 | ord('0')<<16 | ord('1')<<24
        fourcc = (ord("a")) | (ord("v") << 8) | (ord("0") << 16) | (ord("1") << 24)
        mock_cap.get.return_value = fourcc
        mock_videocap.return_value = mock_cap
        video_codec = utils.discover_video_codec("dummy.mp4")
        self.assertEqual(video_codec, "av01")

    def test_discover_video_codec_non_existing_file(self):
        video_codec = utils.discover_video_codec("non_existing_file.mp4")
        self.assertEqual(video_codec, "unknown")

    def test_yolov10_model(self):
        # Test with a valid YOLO v10 model path
        self.assertTrue(is_yolov10_model("/path/to/yolov10s_model.xml"))

    def test_non_yolov10_model(self):
        # Test with a non-YOLO v10 model path
        self.assertFalse(is_yolov10_model("/path/to/other_model.xml"))

    def test_case_insensitivity(self):
        # Test with mixed-case YOLO v10 model path
        self.assertTrue(is_yolov10_model("/path/to/YOLOv10m_model.xml"))

    def test_empty_path(self):
        # Test with an empty string
        self.assertFalse(is_yolov10_model(""))

    def test_no_yolo_in_path(self):
        # Test with a path that does not contain "yolov10"
        self.assertFalse(is_yolov10_model("/path/to/yolo_model.xml"))


if __name__ == "__main__":
    unittest.main()

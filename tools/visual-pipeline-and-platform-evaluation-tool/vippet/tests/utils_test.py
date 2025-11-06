import unittest

import utils


class TestUtils(unittest.TestCase):
    def test_yolov10_model(self):
        # Test with a valid YOLO v10 model path
        self.assertTrue(utils.is_yolov10_model("/path/to/yolov10s_model.xml"))

    def test_non_yolov10_model(self):
        # Test with a non-YOLO v10 model path
        self.assertFalse(utils.is_yolov10_model("/path/to/other_model.xml"))

    def test_case_insensitivity(self):
        # Test with mixed-case YOLO v10 model path
        self.assertTrue(utils.is_yolov10_model("/path/to/YOLOv10m_model.xml"))

    def test_empty_path(self):
        # Test with an empty string
        self.assertFalse(utils.is_yolov10_model(""))

    def test_no_yolo_in_path(self):
        # Test with a path that does not contain "yolov10"
        self.assertFalse(utils.is_yolov10_model("/path/to/yolo_model.xml"))


if __name__ == "__main__":
    unittest.main()

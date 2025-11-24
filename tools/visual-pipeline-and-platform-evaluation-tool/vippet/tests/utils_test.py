import re
import unittest

import utils


class TestUtils(unittest.TestCase):
    def test_generate_unique_id_format(self):
        # Test that the generated ID follows the expected format: prefix-hash
        prefix = "test"
        unique_id = utils.generate_unique_id(prefix)

        # Should start with the prefix
        self.assertTrue(unique_id.startswith(f"{prefix}-"))

        # Should have exactly 8 hexadecimal characters after the prefix and dash
        pattern = rf"^{re.escape(prefix)}-[0-9a-f]{{8}}$"
        self.assertIsNotNone(re.match(pattern, unique_id))

    def test_generate_unique_id_uniqueness(self):
        # Test that multiple calls generate different IDs
        prefix = "pipeline"
        ids = [utils.generate_unique_id(prefix) for _ in range(100)]

        # All IDs should be unique
        self.assertEqual(len(ids), len(set(ids)))

    def test_generate_unique_id_different_prefixes(self):
        # Test that different prefixes produce different IDs
        id1 = utils.generate_unique_id("prefix1")
        id2 = utils.generate_unique_id("prefix2")

        self.assertTrue(id1.startswith("prefix1-"))
        self.assertTrue(id2.startswith("prefix2-"))
        self.assertNotEqual(id1, id2)

    def test_generate_unique_id_empty_prefix(self):
        # Test with an empty prefix
        unique_id = utils.generate_unique_id("")

        # Should start with a dash and have 8 hex characters
        pattern = r"^-[0-9a-f]{8}$"
        self.assertIsNotNone(re.match(pattern, unique_id))

    def test_generate_unique_id_special_chars_prefix(self):
        # Test with special characters in prefix
        prefix = "test_prefix-123"
        unique_id = utils.generate_unique_id(prefix)

        self.assertTrue(unique_id.startswith(f"{prefix}-"))
        pattern = rf"^{re.escape(prefix)}-[0-9a-f]{{8}}$"
        self.assertIsNotNone(re.match(pattern, unique_id))

    def test_generate_unique_id_rapid_succession(self):
        # Test that IDs generated in rapid succession are still unique
        prefix = "rapid"
        ids = []
        for _ in range(10):
            ids.append(utils.generate_unique_id(prefix))
            # No sleep to test rapid generation

        # All should be unique despite being generated rapidly
        self.assertEqual(len(ids), len(set(ids)))

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

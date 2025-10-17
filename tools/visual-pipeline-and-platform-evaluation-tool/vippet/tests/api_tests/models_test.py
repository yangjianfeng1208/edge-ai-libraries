import unittest
from fastapi import FastAPI
from fastapi.testclient import TestClient
from unittest.mock import patch, MagicMock

from api.routes.models import router as models_router


class TestModelsAPI(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        """Set up test client once for all tests."""
        app = FastAPI()
        app.include_router(models_router, prefix="/models")
        cls.client = TestClient(app)

    def _make_model(self, name, display_name, model_type):
        """Helper method to create a mock model."""
        m = MagicMock()
        m.name = name
        m.display_name = display_name
        m.model_type = model_type
        return m

    def test_get_models_returns_models_with_precision(self):
        """Test GET /models returns models with precision extracted from display_name."""
        mock_models = [
            self._make_model(
                "resnet-50-tf_INT8", "ResNet-50 TF (INT8)", "classification"
            ),
            self._make_model("yolov10m", "YOLO v10m 640x640 (FP16)", "detection"),
        ]
        with patch(
            "api.routes.models.SupportedModelsManager"
        ) as MockSupportedModelsManager:
            MockSupportedModelsManager.return_value.get_all_available_models.return_value = mock_models
            response = self.client.get("/models")

            self.assertEqual(response.status_code, 200)
            data = response.json()
            self.assertIsInstance(data, list)
            self.assertEqual(len(data), 2)

            # Check first model
            self.assertEqual(data[0]["name"], "resnet-50-tf_INT8")
            self.assertEqual(data[0]["display_name"], "ResNet-50 TF (INT8)")
            self.assertEqual(data[0]["category"], "classification")
            self.assertEqual(data[0]["precision"], "INT8")

            # Check second model
            self.assertEqual(data[1]["name"], "yolov10m")
            self.assertEqual(data[1]["display_name"], "YOLO v10m 640x640 (FP16)")
            self.assertEqual(data[1]["category"], "detection")
            self.assertEqual(data[1]["precision"], "FP16")

    def test_get_models_returns_models_without_precision(self):
        """Test GET /models returns models without precision when not in display_name."""
        mock_models = [
            self._make_model("mobilenet", "MobileNetV2", "classification"),
        ]
        with patch("api.routes.models.SupportedModelsManager") as MockManager:
            MockManager.return_value.get_all_available_models.return_value = mock_models
            response = self.client.get("/models")

            self.assertEqual(response.status_code, 200)
            data = response.json()
            self.assertEqual(data[0]["name"], "mobilenet")
            self.assertIsNone(data[0]["precision"])

    def test_get_models_empty_list(self):
        """Test GET /models returns empty list when no models available."""
        with patch("api.routes.models.SupportedModelsManager") as MockManager:
            MockManager.return_value.get_all_available_models.return_value = []
            response = self.client.get("/models")

            self.assertEqual(response.status_code, 200)
            data = response.json()
            self.assertEqual(data, [])


if __name__ == "__main__":
    unittest.main()

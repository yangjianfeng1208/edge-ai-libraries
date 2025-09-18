from typing import List, Tuple, Dict, Optional

# Keys for device selection
GPU_0 = "GPU_0"
GPU_N = "GPU_N"
OTHER = "OTHER"
# Placeholder for vaapi_suffix to be replaced at runtime
VAAPI_SUFFIX_PLACEHOLDER = "{vaapi_suffix}"


class PipelineElementSelectionInstructions:
    def __init__(
        self,
        compositor: Optional[Dict[str, List[Tuple[str, str]]]] = None,
        encoder: Optional[Dict[str, List[Tuple[str, str]]]] = None,
        decoder: Optional[Dict[str, List[Tuple[str, str]]]] = None,
        postprocessing: Optional[Dict[str, List[Tuple[str, str]]]] = None,
    ):
        # Each field is a dict: key is GPU_0, GPU_N, or OTHER, value is list of (search, result)
        self.compositor = compositor or {}
        self.encoder = encoder or {}
        self.decoder = decoder or {}
        self.postprocessing = postprocessing or {}


class PipelineElementsSelector:
    def __init__(
        self,
        selection_instructions: PipelineElementSelectionInstructions,
    ):
        self.instructions = selection_instructions

    def select_elements(
        self,
        parameters: dict,
        elements: list,
    ) -> Tuple[Optional[str], Optional[str], Optional[str], Optional[str]]:
        compositor_gpu_id, compositor_vaapi_suffix = self._select_gpu(
            parameters.get("compositor_device", "")
        )
        detection_gpu_id, detection_vaapi_suffix = self._select_gpu(
            parameters.get("object_detection_device", "")
        )

        compositor_element = self._select_element(
            self.instructions.compositor,
            elements,
            compositor_gpu_id,
            compositor_vaapi_suffix,
        )
        encoder_element = self._select_element(
            self.instructions.encoder,
            elements,
            detection_gpu_id,
            detection_vaapi_suffix,
        )
        decoder_element = self._select_element(
            self.instructions.decoder,
            elements,
            detection_gpu_id,
            detection_vaapi_suffix,
        )
        postprocessing_element = self._select_element(
            self.instructions.postprocessing,
            elements,
            detection_gpu_id,
            detection_vaapi_suffix,
        )

        return (
            compositor_element,
            encoder_element,
            decoder_element,
            postprocessing_element,
        )

    @staticmethod
    def _select_gpu(device: str) -> Tuple[int, Optional[str]]:
        gpu_id = -1
        vaapi_suffix = None

        # Determine gpu_id and vaapi_suffix
        # If there is only one GPU, device name is just GPU
        # If there is more than one GPU, device names are like GPU.0, GPU.1, ...
        if device == "GPU":
            gpu_id = 0
        elif device.startswith("GPU."):
            try:
                gpu_index = int(device.split(".")[1])
                if gpu_index == 0:
                    gpu_id = 0
                elif gpu_index > 0:
                    vaapi_suffix = str(128 + gpu_index)
                    gpu_id = gpu_index
            except (IndexError, ValueError):
                gpu_id = -1
        else:
            gpu_id = -1

        return gpu_id, vaapi_suffix

    @staticmethod
    def _select_element(
        field_dict: Dict[str, List[Tuple[str, str]]],
        elements: list,
        gpu_id: int,
        vaapi_suffix: Optional[str],
    ) -> Optional[str]:
        key = OTHER
        if gpu_id == 0:
            key = GPU_0
        elif gpu_id > 0:
            key = GPU_N

        pairs = field_dict.get(key, [])
        # Add OTHER pairs as fallback if key is not OTHER
        if key != OTHER:
            pairs = pairs + field_dict.get(OTHER, [])

        if not pairs:
            return None

        for search, result in pairs:
            if search == "":  # to support optional parameters
                return result

            if VAAPI_SUFFIX_PLACEHOLDER in search or VAAPI_SUFFIX_PLACEHOLDER in result:
                suffix = vaapi_suffix if vaapi_suffix is not None else ""
                search = search.replace(VAAPI_SUFFIX_PLACEHOLDER, suffix)
                result = result.replace(VAAPI_SUFFIX_PLACEHOLDER, suffix)
            for element in elements:
                if element[1] == search:
                    return result
        return None

class PipelineElementsSelector:
    def __init__(self, parameters: dict, elements: list):
        self.parameters = parameters
        self.elements = elements
        self.gpu_id = -1
        self.vaapi_suffix = None

        # Calculate vaapi_suffix once
        device = parameters.get("object_detection_device", "")

        # Determine gpu_id and vaapi_suffix
        # If there is only one GPU, device name is just GPU
        # If there is more than one GPU, device names are like GPU.0, GPU.1, ...
        if device == "GPU":
            self.gpu_id = 0
        elif device.startswith("GPU."):
            try:
                gpu_index = int(device.split(".")[1])
                if gpu_index == 0:
                    self.gpu_id = 0
                elif gpu_index > 0:
                    self.vaapi_suffix = str(128 + gpu_index)
                    self.gpu_id = gpu_index
            except (IndexError, ValueError):
                self.gpu_id = -1
        else:
            self.gpu_id = -1

        self._compositor_element = None
        self._encoder_element = None
        self._decoder_element = None
        self._postprocessing_element = None
        if self.gpu_id > 0:
            varender_compositor = f"varenderD{self.vaapi_suffix}compositor"
            self._compositor_element = next(
                (
                    varender_compositor
                    for element in elements
                    if element[1] == varender_compositor
                ),
                None,
            )

            varender_encoder_lp = f"varenderD{self.vaapi_suffix}h264lpenc"
            varender_encoder = f"varenderD{self.vaapi_suffix}h264enc"
            self._encoder_element = next(
                (
                    varender_encoder_lp
                    for element in elements
                    if element[1] == varender_encoder_lp
                ),
                next(
                    (
                        varender_encoder
                        for element in elements
                        if element[1] == varender_encoder
                    ),
                    None,
                ),
            )

            varender_decoder = f"varenderD{self.vaapi_suffix}h264dec"
            self._decoder_element = next(
                (
                    f"{varender_decoder} ! video/x-raw(memory:VAMemory)"
                    for element in elements
                    if element[1] == varender_decoder
                ),
                None,
            )

            varender_postprocessing = f"varenderD{self.vaapi_suffix}postproc"
            self._postprocessing_element = next(
                (
                    varender_postprocessing
                    for element in elements
                    if element[1] == varender_postprocessing
                ),
                None,
            )
        elif self.gpu_id == 0:
            self._compositor_element = next(
                (
                    "vacompositor"
                    for element in elements
                    if element[1] == "vacompositor"
                ),
                None,
            )

            self._encoder_element = next(
                ("vah264lpenc" for element in elements if element[1] == "vah264lpenc"),
                next(
                    ("vah264enc" for element in elements if element[1] == "vah264enc"),
                    None,
                ),
            )

            self._decoder_element = next(
                (
                    "vah264dec ! video/x-raw(memory:VAMemory)"
                    for element in elements
                    if element[1] == "vah264dec"
                ),
                None,
            )

            self._postprocessing_element = next(
                ("vapostproc" for element in elements if element[1] == "vapostproc"),
                None,
            )

        if self._compositor_element is None:
            self._compositor_element = next(
                ("compositor" for element in elements if element[1] == "compositor"),
                None,
            )

        if self._encoder_element is None:
            self._encoder_element = next(
                (
                    "x264enc bitrate=16000 speed-preset=superfast"
                    for element in elements
                    if element[1] == "x264enc"
                ),
                None,
            )

        if self._decoder_element is None:
            self._decoder_element = next(
                ("decodebin3" for element in elements if element[1] == "decodebin3"),
                None,
            )

        if self._postprocessing_element is None:
            self._postprocessing_element = next(
                ("videoscale" for element in elements if element[1] == "videoscale"),
                None,
            )

    def compositor_element(self):
        return self._compositor_element

    def encoder_element(self):
        return self._encoder_element

    def decoder_element(self):
        return self._decoder_element

    def postprocessing_element(self):
        return self._postprocessing_element

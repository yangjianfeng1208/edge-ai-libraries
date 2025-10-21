/*******************************************************************************
 * Copyright (C) 2025 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 ******************************************************************************/

#pragma once
#include "inference_backend/image.h"
#include "dlstreamer/d3d11/context.h"

#include <functional>
#include <mutex>
#include <set>
#include <stdexcept>
#include <d3d11.h>
#include <dxgi.h>
#include <wrl/client.h>

namespace InferenceBackend {

class D3D11Context {
  public:
    explicit D3D11Context(ID3D11Device* d3d11_device);
    explicit D3D11Context(dlstreamer::ContextPtr d3d11_device_context);

    ~D3D11Context();

    /* getters */
    ID3D11Device* Device() const { return _device.Get(); }
    ID3D11DeviceContext* DeviceContext() const { return _device_context.Get(); }
    ID3D11VideoDevice* VideoDevice() const { return _video_device.Get(); }
    ID3D11VideoContext* VideoContext() const { return _video_context.Get(); }

    // Static mutex for thread safety
    static std::mutex& GetContextMutex();

    /**
     * Lock the GStreamer D3D11 device for thread-safe access.
     * Must be called before any ID3D11DeviceContext or DXGI operations.
     */
    void Lock();

    /**
     * Unlock the GStreamer D3D11 device after operations complete.
     */
    void Unlock();

    void CreateVideoProcessorAndEnumerator(
        uint32_t input_width, uint32_t input_height,
        uint32_t output_width, uint32_t output_height,
        Microsoft::WRL::ComPtr<ID3D11VideoProcessor>& video_processor,
        Microsoft::WRL::ComPtr<ID3D11VideoProcessorEnumerator>& video_processor_enumerator);
    
    bool IsPixelFormatSupported(DXGI_FORMAT format) const;

  private:
    dlstreamer::ContextPtr _device_context_storage;
    Microsoft::WRL::ComPtr<ID3D11Device> _device;
    Microsoft::WRL::ComPtr<ID3D11DeviceContext> _device_context;
    Microsoft::WRL::ComPtr<ID3D11VideoDevice> _video_device;
    Microsoft::WRL::ComPtr<ID3D11VideoContext> _video_context;           
    std::set<DXGI_FORMAT> _supported_pixel_formats;
    GstD3D11Device *_gst_device = nullptr; // GstD3D11Device* for proper locking


    /* private helper methods */
    void create_config_and_contexts();
    void create_supported_pixel_formats();
};

} // namespace InferenceBackend

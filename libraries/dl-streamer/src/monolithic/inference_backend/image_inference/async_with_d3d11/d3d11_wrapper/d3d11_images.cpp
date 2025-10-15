/*******************************************************************************
 * Copyright (C) 2025 Intel Corporation
 *
 * SPDX-License-Identifier: MIT
 ******************************************************************************/

#include "d3d11_images.h"
#include <dxgi.h>

using namespace InferenceBackend;


namespace {

// Forward declaration
DXGI_FORMAT ConvertToDXGIFormat(int pixel_format);

Microsoft::WRL::ComPtr<ID3D11Texture2D> CreateID3D11Texture2D(ID3D11Device* device, uint32_t width, uint32_t height, int pixel_format,  MemoryType memory_type) {
    D3D11_TEXTURE2D_DESC texture2d_desc;
    texture2d_desc.Width = width;
    texture2d_desc.Height = height;
    texture2d_desc.MipLevels = 1;
    texture2d_desc.ArraySize = 1;
    texture2d_desc.Format = ConvertToDXGIFormat(pixel_format);
    texture2d_desc.SampleDesc.Count = 1;
    texture2d_desc.Usage = D3D11_USAGE_DEFAULT;
    texture2d_desc.BindFlags = D3D11_BIND_RENDER_TARGET | D3D11_BIND_SHADER_RESOURCE;
    if (memory_type == MemoryType::SYSTEM)
        texture2d_desc.CPUAccessFlags = D3D11_CPU_ACCESS_READ; // Do we need write too?
    else
        texture2d_desc.CPUAccessFlags = 0;

    Microsoft::WRL::ComPtr<ID3D11Texture2D> texture;
    HRESULT hr = device->CreateTexture2D(&texture2d_desc, nullptr, &texture);
    if (FAILED(hr)) {
        return nullptr;
    }
    return texture;
}

DXGI_FORMAT ConvertToDXGIFormat(int pixel_format) {
    switch (pixel_format) {
        case InferenceBackend::FourCC::FOURCC_BGRA: return DXGI_FORMAT_B8G8R8A8_UNORM;
        case InferenceBackend::FourCC::FOURCC_BGRX: return DXGI_FORMAT_B8G8R8X8_UNORM;
        case InferenceBackend::FourCC::FOURCC_RGBA: return DXGI_FORMAT_R8G8B8A8_UNORM;
        case InferenceBackend::FourCC::FOURCC_NV12: return DXGI_FORMAT_NV12;
        default: return DXGI_FORMAT_UNKNOWN;
    }
}


}

struct Format {
    DXGI_FORMAT dxgi_format;
    InferenceBackend::FourCC ib_fourcc;
};

// This structure contains formats supported by D3D11 video processing, in order of priority.
constexpr Format possible_formats[] = {{DXGI_FORMAT_B8G8R8A8_UNORM, InferenceBackend::FourCC::FOURCC_BGRA},
                                       {DXGI_FORMAT_B8G8R8X8_UNORM, InferenceBackend::FourCC::FOURCC_BGRX},
                                       {DXGI_FORMAT_R8G8B8A8_UNORM, InferenceBackend::FourCC::FOURCC_RGBA},
                                       {DXGI_FORMAT_NV12, InferenceBackend::FourCC::FOURCC_NV12}};

std::string FourccName(int code) {
    const char c1 = (code & (0x000000ff << 24)) >> 24;
    const char c2 = (code & (0x000000ff << 16)) >> 16;
    const char c3 = (code & (0x000000ff << 8)) >> 8;
    const char c4 = code & 0x000000ff;

    return {c4, c3, c2, c1};
}

D3D11Image::D3D11Image() {
    image.d3d11_texture = nullptr;
    image.d3d11_device = nullptr;
}

D3D11Image::D3D11Image(D3D11Context *context_, uint32_t width, uint32_t height, int pixel_format,
                       MemoryType memory_type) {
    if (!context_)
        throw std::invalid_argument("Invalid Vaapi context object");

    context = context_;
    image.type = memory_type;
    image.width = width;
    image.height = height;
    image.format = pixel_format;
    image.d3d11_device = context->Device();
    image.d3d11_texture = CreateID3D11Texture2D(context->Device(), width, height, pixel_format, memory_type).GetAddressOf();
    image_map = std::unique_ptr<ImageMap>(ImageMap::Create(memory_type));
    completed = true;
}

void D3D11Image::Unmap() {
    image_map->Unmap();
}

Image D3D11Image::Map() {
    return image_map->Map(image);
}

D3D11ImagePool::D3D11ImagePool(D3D11Context *context, SizeParams size_params, ImageInfo info) {
    if (!context)
        throw std::invalid_argument("D3D11Context is nullptr");

    if (size_params.size() == 0)
        throw std::invalid_argument("size_params can't be zero");

    if (!context->IsPixelFormatSupported(ConvertToDXGIFormat(info.format))) {
        std::string msg = "Unsupported requested pixel format " + FourccName(info.format) + ". ";
        switch (info.memory_type) {
        case InferenceBackend::MemoryType::SYSTEM: {
            // In the case when the system memory is requested, we can choose the supported format and do software color
            // conversion after.
            bool is_set = false;
            for (auto format : possible_formats)
                if (context->IsPixelFormatSupported(format.dxgi_format)) {
                    msg += "Using a supported format " + FourccName(format.dxgi_format) + ".";
                    info.format = format.ib_fourcc;
                    is_set = true;
                    break;
                }
            if (!is_set)
                throw std::runtime_error(msg + "Could not set the other pixel format, none are supported.");
            else
                GVA_WARNING("%s", msg.c_str());
            break;
        }
        default:
            throw std::runtime_error(msg + "Memory type is not supported to select an alternative pixel format.");
        }
    }

    GVA_INFO("D3D11 image pool size: default=%u, fast=%u", size_params.num_default, size_params.num_fast);

    _images.reserve(size_params.size());
    for (size_t i = 0; i < size_params.size(); i++) {
        _images.push_back(std::unique_ptr<D3D11Image>(
            new D3D11Image(context, info.width, info.height, info.format, info.memory_type)));
    }
}

D3D11Image *D3D11ImagePool::AcquireBuffer() {
    std::unique_lock<std::mutex> lock(_free_images_mutex);
    for (;;) {
        for (auto &image : _images) {
            if (image->completed) {
                image->completed = false;
                return image.get();
            }
        }
        _free_image_condition_variable.wait(lock);
    }
}

void D3D11ImagePool::ReleaseBuffer(D3D11Image *image) {
    if (!image)
        throw std::runtime_error("Received D3D11 image is null");

    image->completed = true;
    _free_image_condition_variable.notify_one();
}

void D3D11ImagePool::Flush() {
    std::unique_lock<std::mutex> lock(_free_images_mutex);
    for (auto &image : _images) {
        if (!image->completed)
            image->sync.wait();
    }
}
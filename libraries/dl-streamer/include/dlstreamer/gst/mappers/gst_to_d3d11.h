#pragma once

#include "dlstreamer/base/memory_mapper.h"
#include "dlstreamer/gst/frame.h"
#include "dlstreamer/d3d11/context.h"
#include "dlstreamer/d3d11/tensor.h"

namespace dlstreamer {

class MemoryMapperGSTToD3D11 : public BaseMemoryMapper {
    constexpr static GstMapFlags GST_MAP_D3D11 = static_cast<GstMapFlags>(GST_MAP_FLAG_LAST << 2);

  public:
    using BaseMemoryMapper::BaseMemoryMapper;

    TensorPtr map(TensorPtr src, AccessMode /*mode*/) override {
        auto src_gst = ptr_cast<GSTTensor>(src);

        // Extract D3D11 texture handle from GstMemory
        void* d3d11_texture_ptr = get_d3d11_texture(src_gst->gst_memory());
        ID3D11Texture2D* d3d11_texture = static_cast<ID3D11Texture2D*>(d3d11_texture_ptr);

        auto ret = std::make_shared<D3D11Tensor>(d3d11_texture, src_gst->plane_index(), src->info(), _output_context);

        ret->set_handle(tensor::key::offset_x, src_gst->offset_x());
        ret->set_handle(tensor::key::offset_y, src_gst->offset_y());
        ret->set_parent(src);
        return ret;
    }

  protected:
    void* get_d3d11_texture(GstMemory *mem) {
        GstMapInfo map_info;
        GstMapFlags flags = GST_MAP_D3D11;
        gboolean sts = gst_memory_map(mem, &map_info, flags);
        if (!sts) { // try with GST_MAP_READ
            flags = static_cast<GstMapFlags>(flags | GST_MAP_READ);
            DLS_CHECK(gst_memory_map(mem, &map_info, flags));
        }
        void* d3d11_texture = *reinterpret_cast<void**>(map_info.data);
        gst_memory_unmap(mem, &map_info);
        return d3d11_texture;
    }
};

} // namespace dlstreamer
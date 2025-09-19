# ==============================================================================
# Copyright (C) 2022-2025 Intel Corporation
#
# SPDX-License-Identifier: MIT
# ==============================================================================

set(TARGET_NAME "openvino_inference")

find_package(OpenCL)
find_package(OpenVINO REQUIRED)
pkg_check_modules(GSTD3D11 gstreamer-d3d11-1.0>=1.16 REQUIRED)
pkg_search_module(VA va libva REQUIRED)

add_library(${TARGET_NAME} OBJECT openvino_inference.cpp)
set_compile_flags(${TARGET_NAME})

target_link_libraries(${TARGET_NAME}
PRIVATE
    openvino::runtime
    dlstreamer_api
    dlstreamer_logger
    ${GSTD3D11_LIBRARIES}
)

target_include_directories(${TARGET_NAME}
PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../monolithic/inference_backend/include/inference_backend/
    ${VA_INCLUDE_DIRS}
    ${GSTD3D11_INCLUDE_DIRS}
)

if (OpenCL_FOUND)
target_include_directories(${TARGET_NAME}
PRIVATE
        ${OpenCL_INCLUDE_DIRS}
)

target_link_libraries(${TARGET_NAME}
PUBLIC
        ${OpenCL_LIBRARY}
)
endif()


install(TARGETS ${TARGET_NAME} DESTINATION ${DLSTREAMER_PLUGINS_INSTALL_PATH})

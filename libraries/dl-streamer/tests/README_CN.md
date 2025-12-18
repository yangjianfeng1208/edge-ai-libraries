# DL Streamer 测试指南

本文档详细说明了 `edge-ai-libraries/libraries/dl-streamer/tests` 目录下的测试如何运行、测试范围以及评判标准。

## 目录结构

```
tests/
├── CMakeLists.txt              # 测试构建配置入口
├── scripts/                    # 测试执行脚本
│   ├── run_unit_tests.sh      # 主测试执行脚本
│   └── unit_test_results.py   # 测试结果解析脚本
└── unit_tests/                # 单元测试目录
    ├── check/                 # C/C++ 单元测试（基于 Google Test）
    │   ├── components/        # 组件测试
    │   ├── elements/          # GStreamer 元素测试
    │   └── pipelines/         # 管道测试
    ├── test_utils/            # 测试工具库
    └── tests_gstgva/          # Python 单元测试（基于 pytest）
        ├── run_tests.py       # Python 测试执行入口
        ├── test_*.py          # 各种 Python 测试文件
        └── utils.py           # 测试辅助函数
```

## 测试类型

### 1. C/C++ 单元测试

位于 `unit_tests/check/` 目录下，使用 **Google Test** 和 **CMake/CTest** 框架。

#### 测试类别：

1. **组件测试** (`check/components/`)
   - `gstvideoanalyticsmeta`: 视频分析元数据测试
   - `gstallocatorwrapper`: 内存分配器包装测试
   - `postprocessing`: 后处理算法测试
   - `classification_history`: 分类历史记录测试
   - `feature_toggler`: 功能开关测试
   - `audio`: 音频处理测试
   - `utils`: 实用工具测试
   - `so_loader`: 动态库加载测试
   - `regular-expression`: 正则表达式测试

2. **元素测试** (`check/elements/`)
   - 测试各种 GStreamer 元素的功能

3. **管道测试** (`check/pipelines/`)
   - 测试完整的 GStreamer 管道流程

#### 构建和执行：
- 通过 CMake 配置：`cmake -DENABLE_TESTS=ON`
- 使用 CTest 执行：`ctest --output-on-failure --verbose`
- 每个测试由 CMakeLists.txt 中的 `add_test()` 命令注册

### 2. Python 单元测试

位于 `unit_tests/tests_gstgva/` 目录下，使用 **pytest** 框架。

#### 主要测试文件：

1. **API 测试**
   - `test_tensor.py`: Tensor 数据结构测试
   - `test_region_of_interest.py`: ROI（感兴趣区域）测试
   - `test_video_frame.py`: 视频帧处理测试
   - `test_audio_event.py`: 音频事件测试
   - `test_audio_frame.py`: 音频帧测试

2. **管道测试**
   - `test_pipeline_color_formats.py`: 颜色格式转换测试
   - `test_pipeline_face_detection_and_classification.py`: 人脸检测和分类管道
   - `test_pipeline_vehicle_pedestrian_tracker.py`: 车辆和行人跟踪
   - `test_pipeline_detection_atss.py`: ATSS 目标检测
   - `test_pipeline_detection_yolo_v10s.py`: YOLOv10 检测
   - `test_pipeline_detection_yolo_11s.py`: YOLO11 检测
   - `test_pipeline_single_human_pose_estimation.py`: 单人姿态估计
   - `test_pipeline_multi_human_pose_estimation.py`: 多人姿态估计
   - `test_pipeline_gvapython.py`: GVA Python 插件测试
   - `test_pipeline_gvapython_vaapi.py`: GVA Python VA-API 测试
   - `test_pipeline_custom_preproc.py`: 自定义预处理

3. **模型测试**
   - 测试各种 OpenVINO 和 ONNX 模型的集成

#### 执行：
- 通过 pytest 执行：`pytest --junitxml=python_tests_results.xml`
- 或使用入口脚本：`python run_tests.py`

## 测试执行方式

### 方法 1: 使用主测试脚本（推荐）

```bash
cd /path/to/dl-streamer
./tests/scripts/run_unit_tests.sh [build_dir] [rebuild_with_coverage] [result_path] [build_type] [timeout_mult]
```

**参数说明：**
- `build_dir`: 构建目录路径（默认：`tests/build`）
- `rebuild_with_coverage`: 是否启用代码覆盖率（默认：`false`）
- `result_path`: 测试结果输出路径（默认：`./ctest_result`）
- `build_type`: 构建类型 Debug/Release（默认：`Debug`）
- `timeout_mult`: 超时倍数（默认：`20`）

**示例：**
```bash
# 基本执行
./tests/scripts/run_unit_tests.sh

# 启用代码覆盖率
./tests/scripts/run_unit_tests.sh ./build true ./results Debug 20
```

### 方法 2: 手动执行 C/C++ 测试

```bash
cd build
ctest --output-on-failure --verbose
# 或者执行特定测试
ctest -R GSTVIDEOANALYTICSMETA_TEST --verbose
```

### 方法 3: 手动执行 Python 测试

```bash
cd tests/unit_tests/tests_gstgva
pytest --junitxml=test_results.xml
# 或执行特定测试
pytest test_video_frame.py -v
```

### 方法 4: 通过 Docker 执行（CI 方式）

```bash
docker run --device /dev/dri --rm \
  --group-add=$(stat -c "%g" /dev/dri/render*) \
  -v /path/to/videos:/home/dlstreamer/video-examples \
  -v /path/to/models:/home/dlstreamer/models \
  -v /path/to/results:/home/dlstreamer/test-results \
  -e VIDEO_EXAMPLES_DIR=/home/dlstreamer/video-examples \
  -e MODELS_PATH=/home/dlstreamer/models \
  -e MODELS_PROC_PATH=/home/dlstreamer/dlstreamer/samples/gstreamer/model_proc \
  dlstreamer-dev-image:latest \
  scripts/run_unit_tests.sh "" "" /home/dlstreamer/test-results
```

## 测试依赖

### 必需环境变量：

1. **MODELS_PATH**: 模型文件路径
   ```bash
   export MODELS_PATH=/path/to/models
   ```

2. **MODELS_PROC_PATH**: 模型处理配置文件路径
   ```bash
   export MODELS_PROC_PATH=/path/to/model_proc
   ```

3. **VIDEO_EXAMPLES_DIR**: 测试视频文件路径
   ```bash
   export VIDEO_EXAMPLES_DIR=/path/to/videos
   ```

### 系统依赖：

- GStreamer 1.16+
- OpenVINO 2024+
- Python 3.x with pytest
- Google Test (用于 C++ 测试)
- Intel GPU 驱动（对于 VA-API 测试）

## 评判标准

### 1. 测试通过标准

所有测试必须满足以下条件才算通过：

#### C/C++ 测试：
- **退出码为 0**: 所有 CTest 测试成功执行
- **无断言失败**: Google Test 断言全部通过
- **无内存泄漏**: 如果启用了内存检查工具
- **无超时**: 测试在规定时间内完成（默认超时 × CK_TIMEOUT_MULTIPLIER）

#### Python 测试：
- **pytest 退出码为 0**: 所有测试通过
- **无异常**: 测试过程中无未捕获异常
- **断言成功**: 所有 assert 语句为真
- **管道正常退出**: GStreamer 管道正常结束，无错误消息

### 2. 测试结果输出

测试结果以 XML 格式输出：

- **CTestResults.xml**: CTest 执行结果
- **ctest-junit.xml**: CTest JUnit 格式报告（CTest 3.21.4+）
- **python_tests_results.xml**: pytest JUnit 格式报告

### 3. CI/CD 评判标准

在持续集成环境中（GitHub Actions），测试按以下标准评判：

#### 成功标准：
- ✅ 所有 C/C++ 单元测试通过
- ✅ 所有 Python 单元测试通过
- ✅ 测试覆盖率达标（如果启用）
- ✅ 无内存泄漏或崩溃
- ✅ 性能测试在可接受范围内

#### 失败处理：
- 测试脚本返回码 1 或 8 视为测试失败但已执行
- 其他返回码视为执行错误
- CI 会生成详细的测试报告和摘要

### 4. 代码覆盖率（可选）

启用代码覆盖率时：
```bash
./tests/scripts/run_unit_tests.sh ./build true ./results Debug
```

生成 HTML 覆盖率报告：
- 位置：`results/code_coverage/index.html`
- 工具：gcovr
- 覆盖范围：排除 thirdparty、tests、samples 目录

## 测试范围

### 功能覆盖：

1. **核心组件**
   - 视频/音频帧处理
   - 元数据管理
   - 张量操作
   - 内存管理

2. **GStreamer 元素**
   - 推理元素（gvainference）
   - 检测元素（gvadetect）
   - 分类元素（gvaclassify）
   - 跟踪元素（gvatrack）
   - Python 元素（gvapython）

3. **AI 模型集成**
   - OpenVINO IR 模型
   - ONNX 模型
   - 各种模型架构：YOLO、SSD、MobileNet、ResNet 等

4. **后处理算法**
   - 目标检测后处理
   - 分类结果处理
   - 姿态估计处理
   - 实例分割处理

5. **硬件加速**
   - CPU 推理
   - GPU 推理（iGPU）
   - VA-API 视频加速

## 测试执行流程（CI）

GitHub Actions 工作流程（`.github/workflows/dls-pr-workflow.yaml`）：

1. **代码扫描阶段**
   - 代码风格检查
   - 许可证头检查
   - 静态代码分析（Coverity）
   - Pylint、Shellcheck、Yamllint
   - Docker 文件检查（Hadolint、Trivy）

2. **构建阶段**
   - 构建 Docker 开发镜像（Ubuntu 22/24）
   - 编译 DL Streamer（Debug 模式）
   - 检查模型和视频文件

3. **测试阶段**
   - 执行 C/C++ 单元测试（CTest）
   - 执行 Python 单元测试（pytest）
   - 收集测试结果
   - 生成测试摘要

4. **报告阶段**
   - 上传测试结果（XML 文件）
   - 生成 GitHub Actions 摘要
   - 归档测试工件

## 调试失败的测试

### 查看详细日志：

```bash
# C++ 测试详细输出
ctest --verbose --output-on-failure

# Python 测试详细输出
pytest -v -s

# 特定测试调试
pytest test_video_frame.py::TestVideoFrame::test_specific_case -v -s
```

### 设置日志级别：

```bash
# GStreamer 调试
export GST_DEBUG=3  # 或更高级别如 4、5
export GST_DEBUG_NO_COLOR=1

# 针对特定插件
export GST_DEBUG=gvadetect:5,gvainference:5
```

### 常见问题：

1. **模型未找到**: 确保 `MODELS_PATH` 正确设置
2. **视频未找到**: 确保 `VIDEO_EXAMPLES_DIR` 正确设置
3. **GPU 访问失败**: 检查 `/dev/dri` 设备权限
4. **超时**: 增加 `CK_TIMEOUT_MULTIPLIER` 值
5. **依赖缺失**: 检查 OpenVINO 和 GStreamer 安装

## 添加新测试

### C++ 测试：

1. 在相应目录创建测试文件（如 `my_test.cpp`）
2. 使用 Google Test 框架编写测试
3. 在 `CMakeLists.txt` 中添加：
   ```cmake
   add_test(NAME MY_TEST COMMAND test_executable)
   ```

### Python 测试：

1. 创建 `test_*.py` 文件
2. 使用 pytest 约定编写测试类和方法
3. 在 `run_tests.py` 中导入并添加到测试套件

## 总结

DL Streamer 的测试基础设施提供了全面的质量保证：

- **多层次测试**: 单元测试、集成测试、管道测试
- **多语言支持**: C/C++ 和 Python 测试
- **自动化执行**: 通过脚本和 CI/CD 自动运行
- **详细报告**: XML 格式的测试结果和覆盖率报告
- **严格标准**: 所有测试必须通过，无异常和错误

通过这套测试体系，确保 DL Streamer 的代码质量、功能正确性和性能稳定性。

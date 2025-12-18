# DL Streamer Test Guide

This document provides a comprehensive guide on how tests in `edge-ai-libraries/libraries/dl-streamer/tests` are executed, what tests are run, and the evaluation criteria used.

## Directory Structure

```
tests/
├── CMakeLists.txt              # Test build configuration entry point
├── scripts/                    # Test execution scripts
│   ├── run_unit_tests.sh      # Main test execution script
│   └── unit_test_results.py   # Test results parsing script
└── unit_tests/                # Unit tests directory
    ├── check/                 # C/C++ unit tests (Google Test-based)
    │   ├── components/        # Component tests
    │   ├── elements/          # GStreamer element tests
    │   └── pipelines/         # Pipeline tests
    ├── test_utils/            # Test utility libraries
    └── tests_gstgva/          # Python unit tests (pytest-based)
        ├── run_tests.py       # Python test entry point
        ├── test_*.py          # Various Python test files
        └── utils.py           # Test helper functions
```

## Test Types

### 1. C/C++ Unit Tests

Located in `unit_tests/check/` directory, using **Google Test** and **CMake/CTest** frameworks.

#### Test Categories:

1. **Component Tests** (`check/components/`)
   - `gstvideoanalyticsmeta`: Video analytics metadata tests
   - `gstallocatorwrapper`: Memory allocator wrapper tests
   - `postprocessing`: Post-processing algorithm tests
   - `classification_history`: Classification history tracking tests
   - `feature_toggler`: Feature toggle tests
   - `audio`: Audio processing tests
   - `utils`: Utility function tests
   - `so_loader`: Shared library loader tests
   - `regular-expression`: Regular expression tests

2. **Element Tests** (`check/elements/`)
   - Tests for various GStreamer elements

3. **Pipeline Tests** (`check/pipelines/`)
   - Tests for complete GStreamer pipeline flows

#### Build and Execution:
- Configure with CMake: `cmake -DENABLE_TESTS=ON`
- Execute with CTest: `ctest --output-on-failure --verbose`
- Each test is registered via `add_test()` command in CMakeLists.txt

### 2. Python Unit Tests

Located in `unit_tests/tests_gstgva/` directory, using **pytest** framework.

#### Main Test Files:

1. **API Tests**
   - `test_tensor.py`: Tensor data structure tests
   - `test_region_of_interest.py`: ROI (Region of Interest) tests
   - `test_video_frame.py`: Video frame processing tests
   - `test_audio_event.py`: Audio event tests
   - `test_audio_frame.py`: Audio frame tests

2. **Pipeline Tests**
   - `test_pipeline_color_formats.py`: Color format conversion tests
   - `test_pipeline_face_detection_and_classification.py`: Face detection and classification pipeline
   - `test_pipeline_vehicle_pedestrian_tracker.py`: Vehicle and pedestrian tracking
   - `test_pipeline_detection_atss.py`: ATSS object detection
   - `test_pipeline_detection_yolo_v10s.py`: YOLOv10 detection
   - `test_pipeline_detection_yolo_11s.py`: YOLO11 detection
   - `test_pipeline_single_human_pose_estimation.py`: Single person pose estimation
   - `test_pipeline_multi_human_pose_estimation.py`: Multi-person pose estimation
   - `test_pipeline_gvapython.py`: GVA Python plugin tests
   - `test_pipeline_gvapython_vaapi.py`: GVA Python VA-API tests
   - `test_pipeline_custom_preproc.py`: Custom preprocessing tests

3. **Model Tests**
   - Tests for various OpenVINO and ONNX model integrations

#### Execution:
- Via pytest: `pytest --junitxml=python_tests_results.xml`
- Or using entry script: `python run_tests.py`

## Test Execution Methods

### Method 1: Using Main Test Script (Recommended)

```bash
cd /path/to/dl-streamer
./tests/scripts/run_unit_tests.sh [build_dir] [rebuild_with_coverage] [result_path] [build_type] [timeout_mult]
```

**Parameter Description:**
- `build_dir`: Build directory path (default: `tests/build`)
- `rebuild_with_coverage`: Enable code coverage (default: `false`)
- `result_path`: Test results output path (default: `./ctest_result`)
- `build_type`: Build type Debug/Release (default: `Debug`)
- `timeout_mult`: Timeout multiplier (default: `20`)

**Examples:**
```bash
# Basic execution
./tests/scripts/run_unit_tests.sh

# Enable code coverage
./tests/scripts/run_unit_tests.sh ./build true ./results Debug 20
```

### Method 2: Manual C/C++ Test Execution

```bash
cd build
ctest --output-on-failure --verbose
# Or run specific test
ctest -R GSTVIDEOANALYTICSMETA_TEST --verbose
```

### Method 3: Manual Python Test Execution

```bash
cd tests/unit_tests/tests_gstgva
pytest --junitxml=test_results.xml
# Or run specific test
pytest test_video_frame.py -v
```

### Method 4: Docker Execution (CI Method)

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

## Test Dependencies

### Required Environment Variables:

1. **MODELS_PATH**: Path to model files
   ```bash
   export MODELS_PATH=/path/to/models
   ```

2. **MODELS_PROC_PATH**: Path to model processing configuration files
   ```bash
   export MODELS_PROC_PATH=/path/to/model_proc
   ```

3. **VIDEO_EXAMPLES_DIR**: Path to test video files
   ```bash
   export VIDEO_EXAMPLES_DIR=/path/to/videos
   ```

### System Dependencies:

- GStreamer 1.16+
- OpenVINO 2024+
- Python 3.x with pytest
- Google Test (for C++ tests)
- Intel GPU drivers (for VA-API tests)

## Evaluation Criteria

### 1. Test Pass Criteria

All tests must meet the following conditions to pass:

#### C/C++ Tests:
- **Exit code 0**: All CTest tests executed successfully
- **No assertion failures**: All Google Test assertions pass
- **No memory leaks**: If memory checking tools are enabled
- **No timeouts**: Tests complete within specified time (default timeout × CK_TIMEOUT_MULTIPLIER)

#### Python Tests:
- **pytest exit code 0**: All tests pass
- **No exceptions**: No uncaught exceptions during test execution
- **Assertions succeed**: All assert statements are true
- **Pipeline exits normally**: GStreamer pipelines end without errors

### 2. Test Result Output

Test results are output in XML format:

- **CTestResults.xml**: CTest execution results
- **ctest-junit.xml**: CTest JUnit format report (CTest 3.21.4+)
- **python_tests_results.xml**: pytest JUnit format report

### 3. CI/CD Evaluation Criteria

In continuous integration environments (GitHub Actions), tests are evaluated by:

#### Success Criteria:
- ✅ All C/C++ unit tests pass
- ✅ All Python unit tests pass
- ✅ Code coverage meets threshold (if enabled)
- ✅ No memory leaks or crashes
- ✅ Performance tests within acceptable range

#### Failure Handling:
- Test script return codes 1 or 8 indicate tests ran but some failed
- Other return codes indicate execution errors
- CI generates detailed test reports and summaries

### 4. Code Coverage (Optional)

To enable code coverage:
```bash
./tests/scripts/run_unit_tests.sh ./build true ./results Debug
```

HTML coverage report generated:
- Location: `results/code_coverage/index.html`
- Tool: gcovr
- Coverage scope: Excludes thirdparty, tests, and samples directories

## Test Coverage

### Functional Coverage:

1. **Core Components**
   - Video/audio frame processing
   - Metadata management
   - Tensor operations
   - Memory management

2. **GStreamer Elements**
   - Inference elements (gvainference)
   - Detection elements (gvadetect)
   - Classification elements (gvaclassify)
   - Tracking elements (gvatrack)
   - Python elements (gvapython)

3. **AI Model Integration**
   - OpenVINO IR models
   - ONNX models
   - Various architectures: YOLO, SSD, MobileNet, ResNet, etc.

4. **Post-processing Algorithms**
   - Object detection post-processing
   - Classification result processing
   - Pose estimation processing
   - Instance segmentation processing

5. **Hardware Acceleration**
   - CPU inference
   - GPU inference (iGPU)
   - VA-API video acceleration

## Test Execution Workflow (CI)

GitHub Actions workflow (`.github/workflows/dls-pr-workflow.yaml`):

1. **Code Scanning Phase**
   - Code style checks
   - License header checks
   - Static code analysis (Coverity)
   - Pylint, Shellcheck, Yamllint
   - Dockerfile checks (Hadolint, Trivy)

2. **Build Phase**
   - Build Docker development images (Ubuntu 22/24)
   - Compile DL Streamer (Debug mode)
   - Verify models and video files

3. **Test Phase**
   - Execute C/C++ unit tests (CTest)
   - Execute Python unit tests (pytest)
   - Collect test results
   - Generate test summary

4. **Report Phase**
   - Upload test results (XML files)
   - Generate GitHub Actions summary
   - Archive test artifacts

## Debugging Failed Tests

### View Detailed Logs:

```bash
# C++ test verbose output
ctest --verbose --output-on-failure

# Python test verbose output
pytest -v -s

# Debug specific test
pytest test_video_frame.py::TestVideoFrame::test_specific_case -v -s
```

### Set Log Levels:

```bash
# GStreamer debugging
export GST_DEBUG=3  # Or higher levels like 4, 5
export GST_DEBUG_NO_COLOR=1

# For specific plugins
export GST_DEBUG=gvadetect:5,gvainference:5
```

### Common Issues:

1. **Models not found**: Ensure `MODELS_PATH` is correctly set
2. **Videos not found**: Ensure `VIDEO_EXAMPLES_DIR` is correctly set
3. **GPU access failed**: Check `/dev/dri` device permissions
4. **Timeout**: Increase `CK_TIMEOUT_MULTIPLIER` value
5. **Missing dependencies**: Verify OpenVINO and GStreamer installation

## Adding New Tests

### C++ Tests:

1. Create test file in appropriate directory (e.g., `my_test.cpp`)
2. Write tests using Google Test framework
3. Add to `CMakeLists.txt`:
   ```cmake
   add_test(NAME MY_TEST COMMAND test_executable)
   ```

### Python Tests:

1. Create `test_*.py` file
2. Write test classes and methods following pytest conventions
3. Import and add to test suite in `run_tests.py`

## Summary

DL Streamer's test infrastructure provides comprehensive quality assurance:

- **Multi-level testing**: Unit tests, integration tests, pipeline tests
- **Multi-language support**: C/C++ and Python tests
- **Automated execution**: Via scripts and CI/CD pipelines
- **Detailed reporting**: XML-formatted test results and coverage reports
- **Strict criteria**: All tests must pass without exceptions or errors

This test system ensures DL Streamer's code quality, functional correctness, and performance stability.

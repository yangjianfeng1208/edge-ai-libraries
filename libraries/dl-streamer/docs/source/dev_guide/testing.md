# Testing Guide

This guide explains how to run tests for Deep Learning Streamer (DL Streamer), understand test results, and the standards used for quality assurance.

## Overview

DL Streamer uses a comprehensive testing framework to ensure code quality and reliability. The test suite includes:

1. **Unit Tests (C++)** - Component-level tests using Google Test framework
2. **Integration Tests (Python)** - End-to-end pipeline tests using pytest

Tests are automatically executed in CI/CD pipelines and can also be run locally for development.

## Test Types

### 1. Unit Tests (CTest)

Unit tests are written in C++ using the Google Test (gtest) framework and executed through CMake's CTest runner. These tests validate individual components, elements, and utilities.

**Test Categories:**
- **Components Tests** - Core functionality (preprocessing, postprocessing, metadata, etc.)
- **Elements Tests** - GStreamer element behavior (inference, classification, detection, etc.)
- **Pipeline Tests** - Complete pipeline scenarios

**Location:** `tests/unit_tests/check/`

**Key Components Tested:**
- Video analytics metadata (`gstvideoanalyticsmeta`)
- Pre-processing and post-processing modules
- Audio event and frame handling
- Object detection and classification elements
- Feature toggling and configuration
- Memory management and allocators

### 2. Integration Tests (pytest)

Integration tests are written in Python using pytest and validate complete pipeline execution with real models and media files.

**Location:** `tests/unit_tests/tests_gstgva/`

**Test Coverage:**
- Face detection and recognition pipelines
- Object detection with various YOLO models
- Color format conversions
- Custom preprocessing
- Audio analytics pipelines
- Multi-stage inference workflows

## Running Tests Locally

### Prerequisites

Before running tests, ensure you have:

1. **Built DL Streamer** - Follow the [compilation guide](./advanced_install/advanced_install_guide_compilation.md)
2. **Required Models** - Set `MODELS_PATH` environment variable
3. **Test Videos** - Set `VIDEO_EXAMPLES_DIR` environment variable
4. **OpenVINO** - Properly configured and sourced

### Running All Tests

Use the provided test script:

```bash
cd libraries/dl-streamer
./tests/scripts/run_unit_tests.sh [build_dir] [rebuild_with_code_coverage] [result_path] [build_type] [timeout_mult]
```

**Parameters:**
- `build_dir` - Build directory (default: `tests/build`)
- `rebuild_with_code_coverage` - Enable code coverage (default: `false`)
- `result_path` - Directory for test results (default: `./ctest_result`)
- `build_type` - Build type: `Debug` or `Release` (default: `Debug`)
- `timeout_mult` - Timeout multiplier for GPU tests (default: `20`)

**Example:**
```bash
# Set required environment variables
export MODELS_PATH=/path/to/models
export MODELS_PROC_PATH=/path/to/model_proc
export VIDEO_EXAMPLES_DIR=/path/to/test/videos
source /opt/intel/openvino_2024/setupvars.sh

# Run tests
./tests/scripts/run_unit_tests.sh ./build false ./test-results Debug 20
```

### Running Specific Test Categories

**Run only C++ unit tests:**
```bash
cd build
ctest -T Test --output-on-failure --verbose
```

**Run only Python integration tests:**
```bash
cd tests/unit_tests/tests_gstgva
py.test --junitxml=python_tests_results.xml
```

**Run specific test by name:**
```bash
cd build
ctest -R test_name --verbose
```

**Run tests matching a pattern:**
```bash
cd build
ctest -R "test_.*detection" --verbose
```

## CI/CD Test Execution

Tests are automatically executed on every pull request through GitHub Actions workflows.

### CI Workflow

The main workflow (`.github/workflows/dls-pr-workflow.yaml`) triggers:

1. **Code Quality Scans**
   - Code style checks
   - License header validation
   - Static code analysis (Coverity)
   - Security scans (Trivy, ClamAV, Bandit)
   - Linting (pylint, shellcheck, yamllint)

2. **Build and Test Jobs**
   - Build development Docker images (Ubuntu 22.04 and 24.04)
   - Run unit tests inside containers
   - Build and test DEB packages
   - Build and test Windows DLLs

### Test Environment

Tests run in Docker containers based on:
- **Ubuntu 22.04** - `docker/ubuntu/ubuntu22.Dockerfile`
- **Ubuntu 24.04** - `docker/ubuntu/ubuntu24.Dockerfile`
- **Testing image** - `docker/ubuntu/ubuntu-testing.Dockerfile`

**Environment variables in CI:**
```bash
MODELS_PATH=/home/runner/models
VIDEO_EXAMPLES_DIR=/home/runner/video-examples
MODELS_PROC_PATH=/home/dlstreamer/dlstreamer/samples/gstreamer/model_proc
```

### CI Test Execution Flow

1. Build development Docker image with `BUILD_ARG=Debug`
2. Mount required directories (models, videos, test scripts)
3. Execute `scripts/run_unit_tests.sh` inside container
4. Generate test result XMLs (CTest and pytest)
5. Parse results using `unit_test_results.py`
6. Upload test artifacts to GitHub

## Understanding Test Results

### Test Output Files

After running tests, the following files are generated in the results directory:

1. **ctest-junit.xml** - CTest results in JUnit format
2. **python_tests_results.xml** - pytest results in JUnit format
3. **CTestResults.xml** - Detailed CTest results
4. **unit_test_summary.txt** - Human-readable summary

### Test Summary Format

The `unit_test_results.py` script generates a summary with:

```
CTest: Total: X, Passed: Y, Failed: Z, Errors: E, Skipped: S
Failed tests (CTest):
    - test_name_1
    - test_name_2

Pytest: Total: X, Passed: Y, Failed: Z, Errors: E, Skipped: S
Failed tests (Pytest):
    - test_name_1
    - test_name_2
```

### Pass/Fail Criteria

**Test Suite Passes When:**
- All required tests execute successfully (exit code 0)
- No test failures or errors
- Test crashes are treated as failures

**Acceptable Exit Codes:**
- `0` - All tests passed
- `1` - Tests ran but some failed (handled by CI)
- `8` - Tests ran but some failed (handled by CI)

The CI workflow accepts exit codes 1 and 8 as "tests completed" and relies on the XML output to determine actual pass/fail status.

### Interpreting Results in CI

**GitHub Actions Summary** displays:
- Test Summary for Unit Tests on platform
- Total/Passed/Failed/Errors/Skipped counts
- List of failed test names

**Artifacts:** Test result XMLs are uploaded for detailed analysis.

## Test Standards and Best Practices

### Test Coverage Standards

- **Unit Tests**: Should cover individual functions and components
- **Integration Tests**: Should validate complete workflows
- **Regression Tests**: All bug fixes should include tests

### Test Quality Requirements

1. **Deterministic** - Tests must produce consistent results
2. **Isolated** - Tests should not depend on execution order
3. **Fast** - Unit tests should complete quickly (< 1 second each)
4. **Clear** - Test names should describe what is being tested
5. **Maintainable** - Tests should be easy to understand and update

### Timeout Configuration

- Default timeout multiplier: `20x` (configured via `CK_TIMEOUT_MULTIPLIER`)
- GPU tests may require longer timeouts
- Adjustable via script parameter

### Required Test Environments

Tests must pass on:
- Ubuntu 22.04 with OpenVINO 2024
- Ubuntu 24.04 with OpenVINO 2024
- Intel CPU, GPU, and VPU platforms

## Debugging Test Failures

### Viewing Test Output

**Enable verbose output:**
```bash
export GST_DEBUG=3
export GST_DEBUG_NO_COLOR=1
ctest -V
```

**Run single test with debugging:**
```bash
cd build
./tests/unit_tests/check/elements/inference/test_inference --gtest_filter=TestName
```

### Common Issues

1. **Missing Models**
   - Ensure `MODELS_PATH` points to required model files
   - Check `MODELS_PROC_PATH` for model configuration files

2. **Missing Media Files**
   - Verify `VIDEO_EXAMPLES_DIR` contains test videos
   - Check file permissions

3. **GPU Tests Failing**
   - Increase timeout multiplier
   - Verify GPU drivers are installed (`vainfo`)
   - Check device permissions (`/dev/dri/render*`)

4. **Environment Issues**
   - Source OpenVINO: `source /opt/intel/openvino_2024/setupvars.sh`
   - Set library paths: `LD_LIBRARY_PATH`, `GST_PLUGIN_PATH`

### Analyzing Failed Tests

1. Check test logs in results directory
2. Review JUnit XML files for detailed error messages
3. Run failed test individually with verbose output
4. Verify test environment variables are set correctly

## Code Coverage

To generate code coverage reports:

```bash
./tests/scripts/run_unit_tests.sh ./build true ./coverage-results Debug 20
```

Coverage reports are generated in HTML format:
- Location: `coverage-results/code_coverage/index.html`
- Excludes: thirdparty, tests, samples directories

## Adding New Tests

### Adding C++ Unit Tests

1. Create test file in appropriate directory under `tests/unit_tests/check/`
2. Add to CMakeLists.txt
3. Use Google Test macros (`TEST`, `TEST_F`, etc.)
4. Register test with CTest: `add_test(NAME test_name COMMAND test_executable)`

**Example:**
```cpp
#include <gtest/gtest.h>

TEST(ComponentName, TestDescription) {
    // Arrange
    // Act
    // Assert
    EXPECT_EQ(expected, actual);
}
```

### Adding Python Integration Tests

1. Create test file in `tests/unit_tests/tests_gstgva/`
2. Use pytest conventions (`test_*.py`, `def test_*()`)
3. Use pipeline runner utilities for consistency

**Example:**
```python
import pytest
from pipeline_runner import run_pipeline

def test_detection_pipeline():
    pipeline = "filesrc location=test.mp4 ! ..."
    result = run_pipeline(pipeline)
    assert result.success
    assert result.detected_objects > 0
```

## Related Documentation

- [Advanced Installation Guide](./advanced_install/advanced_install_guide_compilation.md)
- [Coding Style](./coding_style.md)
- [How to Contribute](./how_to_contribute.md)
- [Performance Guide](./performance_guide.md)

## Troubleshooting

For test-related issues:
1. Review test logs and error messages
2. Check environment variable configuration
3. Verify model and media file availability
4. Consult [GitHub Issues](https://github.com/open-edge-platform/edge-ai-libraries/issues)
5. Use [GitHub Discussions](https://github.com/open-edge-platform/edge-ai-libraries/discussions) for questions

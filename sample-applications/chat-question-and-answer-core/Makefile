# SPDX-FileCopyrightText: 2025 Intel Corporation
# SPDX-License-Identifier: Apache-2.0

# Safe shell invocations
SHELL := bash -eu -o pipefail

# Default goal to show help
.DEFAULT_GOAL := help

# --- 1. USER CONFIGURATION ---
# ------------------------------------------------------------------------------
# Edit the variables in this section for your microservice.
# ------------------------------------------------------------------------------

# The base name of the microservice. Used as a prefix for Docker image names.
SERVICE_NAME := chatqna-core

# The version tag for the Docker images. Defaults to 'latest'.
# CI can override this from the command line: make build VERSION=...
VERSION ?= latest

# --- Component Configuration ---

# 1. List all the component names you want to build.
#    Example: COMPONENTS := frontend backend api worker
COMPONENTS := frontend backend

# 2. For EACH component listed above, provide its context directory and Dockerfile path.
#    Follow the naming convention: $(COMPONENT_NAME)_CONTEXT_DIR and $(COMPONENT_NAME)_DOCKERFILE

# -- Frontend Component --
frontend_CONTEXT_DIR := ./ui/
frontend_DOCKERFILE  := ./ui/Dockerfile

# -- Backend Component --
backend_CONTEXT_DIR := ./
backend_DOCKERFILE  := ./docker/Dockerfile

# Add more component configurations here if needed, for example:
# -- API Component --
# api_CONTEXT_DIR := ./api/
# api_DOCKERFILE  := ./api/Dockerfile

# 3. (Optional) List all components that have unit tests.
#    If a component has tests, its name must also be in the main COMPONENTS list.
TEST_COMPONENTS := frontend backend

# Required versions for CI/CD and local development
PYTHON_VERSION := 3.12
NODE_VERSION   := 22

# ------------------------------------------------------------------------------
# DO NOT EDIT BELOW THIS LINE
# ------------------------------------------------------------------------------

# --- 2. INTERNAL LOGIC & SETUP ---

# Dynamically generate the list of build and test targets
BUILD_TARGETS := $(addprefix build-,$(COMPONENTS))
TEST_TARGETS  := $(addprefix test-,$(TEST_COMPONENTS))

# All targets are actions, not files.
.PHONY: all build test clean help list shellcheck pylint $(BUILD_TARGETS) $(TEST_TARGETS) get-scan-matrix-json get-python-version get-node-version

# --- 3. CORE BUILD TARGETS ---

# The main build command.
build: $(BUILD_TARGETS)
	@if [ -z "$(BUILD_TARGETS)" ]; then \
		echo "🔴 ERROR: No components are defined in the Makefile. Please list them in the 'COMPONENTS' variable."; \
		exit 1; \
	else \
		echo "✅ All defined components built successfully."; \
	fi

# Dynamically generate a build rule for each component
$(foreach component,$(COMPONENTS), \
    $(eval .PHONY: build-$(component)) \
    $(eval build-$(component): ; \
        @echo "--- Building $(component) image: $(SERVICE_NAME)-$(component):$(VERSION) ---"; \
        docker build \
            -t $(SERVICE_NAME)-$(component):$(VERSION) \
            -f $($(component)_DOCKERFILE) $($(component)_CONTEXT_DIR) \
    ) \
)

# --- 4. UNIT TEST TARGETS ---

# The main test command.
test: $(TEST_TARGETS)
	@if [ -z "$(TEST_TARGETS)" ]; then \
		echo "🟡 WARNING: No components are enabled for testing in the Makefile."; \
	else \
		echo "✅ All enabled unit tests completed."; \
	fi

# --- Specific Test Recipes ---
# You still need to provide the specific test commands for each component that has tests.

# Recipe to run frontend unit tests.
.PHONY: test-frontend
test-frontend:
	@echo "--- Running frontend unit tests in [$(frontend_CONTEXT_DIR)] ---"
	@(cd $(frontend_CONTEXT_DIR) && npm install && npm run coverage) || true

# Recipe to run backend unit tests.
.PHONY: test-backend
test-backend:
	@echo "--- Running backend unit tests ---"
	@python$(PYTHON_VERSION) -m venv chat-qna-core-venv; \
	source chat-qna-core-venv/bin/activate; \
	pip install poetry; \
	poetry install --with dev || true; \
	poetry add --group dev pytest-cov || true; \
	poetry run pytest tests/ \
		--cov=app \
		--cov-report=html:coverage-backend \
		--cov-report=xml:coverage-backend.xml \
		--cov-report=term-missing \
		--cov-branch || true; \
	deactivate; \
	rm -rf chat-qna-core-venv;

# --- 5. LINTING & SECURITY TARGETS ---

# Recipe to scan all shell scripts in the project with ShellCheck.
.PHONY: shellcheck
shellcheck:
	@echo "--- Running ShellCheck to scan for shell script issues ---"
	@mkdir -p security-results
	@echo "=== ShellCheck Scan Results ===" > security-results/shellcheck-report-$(SERVICE_NAME).txt
	@echo "Scan Date: $$(date)" >> security-results/shellcheck-report-$(SERVICE_NAME).txt
	@echo "" >> security-results/shellcheck-report-$(SERVICE_NAME).txt
	@find . -type f \( -name "*.sh" -o -name "*.bash" \) -print0 | while IFS= read -r -d '' file; do \
		echo "Checking: $$file" >> security-results/shellcheck-report-$(SERVICE_NAME).txt; \
		shellcheck "$$file" >> security-results/shellcheck-report-$(SERVICE_NAME).txt 2>&1 || true; \
		echo "---" >> security-results/shellcheck-report-$(SERVICE_NAME).txt; \
	done
	@echo "✅ ShellCheck scan completed. Report generated at security-results/shellcheck-report-$(SERVICE_NAME).txt"

.PHONY: pylint
pylint:
	@echo "--- Setting up environment and running Pylint to analyze Python code ---"
	@python$(PYTHON_VERSION) -m venv pylint-venv; \
	source pylint-venv/bin/activate; \
	pip install poetry; \
	poetry install --with dev || true; \
	poetry add --group dev pylint || true; \
	\
	echo "--- Generating Pylint report ---"; \
	mkdir -p security-results; \
	echo "=== Pylint Scan Results ===" > security-results/pylint-report-$(SERVICE_NAME).txt; \
	echo "Scan Date: $$(date)" >> security-results/pylint-report-$(SERVICE_NAME).txt; \
	echo "" >> security-results/pylint-report-$(SERVICE_NAME).txt; \
	\
	( \
		echo "[MESSAGES CONTROL]"; \
		echo "disable=C0111,C0103,R0903,R0913,W0613,W0622,R0801,R0902,R0914,R0915,R0912,C0301,C0302"; \
		echo ""; \
		echo "[FORMAT]"; \
		echo "max-line-length=120"; \
		echo ""; \
		echo "[REPORTS]"; \
		echo "output-format=text"; \
		echo "reports=yes"; \
	) > .pylintrc; \
	\
	find app/ -type f -name "*.py" -exec poetry run pylint --rcfile=.pylintrc {} + >> security-results/pylint-report-$(SERVICE_NAME).txt 2>&1 || true; \
	\
	echo "--- Cleaning up Pylint environment ---"; \
	deactivate; \
	rm -rf pylint-venv; \
	rm -f .pylintrc; \
	@echo "✅ Pylint scan completed. Report generated at security-results/pylint-report-$(SERVICE_NAME).txt"

# --- 6. GITHUB ACTIONS EXPORT TARGETS ---

# Generates a JSON array of component objects for the CI matrix
get-scan-matrix-json:
	@printf '{"include":['
	@$(foreach component,$(COMPONENTS), \
		$(eval COMMA := $(if $(findstring $(component),$(firstword $(COMPONENTS))),,',')) \
		printf '%s' '$(COMMA){"name":"$(component)","image":"$(SERVICE_NAME)-$(component):$(VERSION)","context":"$($(component)_CONTEXT_DIR)","dockerfile":"$($(component)_DOCKERFILE)"}'; \
	)
	@printf ']}\n'

# --- UTILITY TARGETS ---

list:
	@echo "=================================================="
	@echo " Build configuration for service: '$(SERVICE_NAME)'"
	@echo " Version tag: '$(VERSION)'"
	@echo "=================================================="
	@echo "Enabled components to build:"
	@$(foreach component,$(COMPONENTS), \
		echo "  - $(component)"; \
	)
	@echo "Docker images to be built:"
	@$(foreach component,$(COMPONENTS), \
		echo "  - $(SERVICE_NAME)-$(component):$(VERSION)"; \
	)
	@echo "Enabled components to test:"
	@$(foreach component,$(TEST_COMPONENTS), \
		echo "  - $(component)"; \
	)

clean:
	@echo "Cleaning up Docker images for service '$(SERVICE_NAME)'..."
	@$(foreach component,$(COMPONENTS), \
		docker rmi -f $(SERVICE_NAME)-$(component):$(VERSION); \
	)

help:
	@echo "Usage: make [target]"
	@echo ""
	@echo "Available targets for service '$(SERVICE_NAME)':"
	@echo "  build       - Build all enabled Docker images."
	@echo "  test        - Run all enabled unit tests."
	@echo "  shellcheck  - Run ShellCheck to find issues in shell scripts."
	@echo "  pylint      - Run Pylint to analyze Python code." # <-- ADD THIS LINE
	@echo "  list        - Show the build configuration for this service."
	@echo "  clean       - Remove Docker images built by this Makefile."
	@echo "  help        - Show this help message."
	@echo ""
	@echo "Individual component targets:"
	@$(foreach component,$(COMPONENTS), \
		echo "  build-$(component) - Build the $(component) component."; \
	)
	@$(foreach component,$(TEST_COMPONENTS), \
		echo "  test-$(component) - Test the $(component) component."; \
	)

# --- 6. GITHUB ACTIONS EXPORT TARGETS ---
# These targets help the CI pipeline read configuration values dynamically.

get-service-name:
	@echo $(SERVICE_NAME)

get-component-names:
	@echo "$(COMPONENTS)"

get-image-tags:
	@$(foreach component,$(COMPONENTS), \
		echo "$(SERVICE_NAME)-$(component):$(VERSION)"; \
	)

get-context-dirs:
	@$(foreach component,$(COMPONENTS), \
		echo "$($(component)_CONTEXT_DIR)"; \
	)

get-python-version:
	@echo $(PYTHON_VERSION)

get-node-version:
	@echo $(NODE_VERSION)
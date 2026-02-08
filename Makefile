# ============================================================================
# End-to-End RL UAV Flight Controller — Common Development Commands
# ============================================================================
# Usage: make <target>
# Run `make help` to see all available targets.
# ============================================================================

.DEFAULT_GOAL := help
SHELL := /bin/bash

# ── Python ──────────────────────────────────────────────────────────────────
.PHONY: install install-dev install-sim install-train install-docs

install:  ## Install base project dependencies
	pip install -e .

install-dev:  ## Install development dependencies (lint, test, docs)
	pip install -e ".[dev]"

install-sim:  ## Install simulation dependencies
	pip install -e ".[sim]"

install-train:  ## Install RL training dependencies
	pip install -e ".[train]"

install-docs:  ## Install documentation dependencies
	pip install -e ".[docs]"

# ── Linting & Formatting ───────────────────────────────────────────────────
.PHONY: lint format typecheck lint-all

lint:  ## Run ruff linter
	ruff check simulation/ training/ tests/

format:  ## Run ruff formatter
	ruff format simulation/ training/ tests/

typecheck:  ## Run mypy type checker
	mypy simulation/ training/ --ignore-missing-imports

lint-all: lint typecheck  ## Run all Python linting

# ── Testing ─────────────────────────────────────────────────────────────────
.PHONY: test test-cov test-sim test-fw

test:  ## Run all Python tests
	pytest tests/ simulation/ training/ -v --tb=short

test-cov:  ## Run tests with coverage report
	pytest tests/ simulation/ training/ \
		--cov=simulation --cov=training \
		--cov-report=term-missing --cov-report=html

test-sim:  ## Run simulation-specific tests
	pytest tests/ simulation/ -v --tb=short -m "not hardware"

test-fw:  ## Run firmware tests (placeholder — requires QEMU)
	@echo "Firmware tests not yet configured. Will use Ceedling + QEMU."

# ── Documentation ───────────────────────────────────────────────────────────
.PHONY: docs docs-serve docs-build

docs: docs-serve  ## Alias for docs-serve

docs-serve:  ## Start local MkDocs development server
	mkdocs serve

docs-build:  ## Build MkDocs static site
	mkdocs build --strict

# ── Requirements (Doorstop) ─────────────────────────────────────────────────
.PHONY: reqs reqs-publish reqs-validate

reqs: reqs-validate  ## Validate Doorstop requirements

reqs-validate:  ## Validate requirements traceability
	doorstop

reqs-publish:  ## Publish requirements as HTML
	@mkdir -p docs/requirements/output
	doorstop publish all docs/requirements/output/

# ── Pre-commit ──────────────────────────────────────────────────────────────
.PHONY: pre-commit pre-commit-install

pre-commit:  ## Run pre-commit on all files
	pre-commit run --all-files

pre-commit-install:  ## Install pre-commit git hooks
	pre-commit install

# ── Docker ──────────────────────────────────────────────────────────────────
.PHONY: docker-sim docker-fw docker-up docker-down

docker-sim:  ## Build simulation Docker image
	docker build -f tools/docker/Dockerfile.sim -t uav-sim .

docker-fw:  ## Build firmware Docker image
	docker build -f tools/docker/Dockerfile.firmware -t uav-firmware .

docker-up:  ## Start all Docker services
	cd tools/docker && docker-compose up -d

docker-down:  ## Stop all Docker services
	cd tools/docker && docker-compose down

# ── Clean ───────────────────────────────────────────────────────────────────
.PHONY: clean clean-build clean-test clean-docs

clean: clean-build clean-test clean-docs  ## Remove all generated files

clean-build:  ## Remove build artifacts
	rm -rf build/ dist/ *.egg-info .eggs/

clean-test:  ## Remove test artifacts
	rm -rf .pytest_cache/ htmlcov/ .coverage coverage.xml

clean-docs:  ## Remove built documentation
	rm -rf site/

# ── Help ────────────────────────────────────────────────────────────────────
.PHONY: help

help:  ## Show this help message
	@echo "Usage: make [target]"
	@echo ""
	@echo "Targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-20s\033[0m %s\n", $$1, $$2}'

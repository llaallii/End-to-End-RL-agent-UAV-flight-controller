# Tests

System-level and integration tests spanning multiple subsystems.

## Structure

```
tests/
├── integration/     # Cross-subsystem integration tests
├── system/          # End-to-end system tests
├── performance/     # Benchmarks and performance tests
├── safety/          # Safety-critical test cases
└── conftest.py      # Shared pytest fixtures
```

Unit tests for each subsystem live in their respective directories
(`simulation/`, `firmware/tests/`, `training/`).

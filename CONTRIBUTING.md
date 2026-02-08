# Contributing to End-to-End RL UAV Flight Controller

## Branching Strategy

This project uses a simplified Git Flow adapted for solo development:

```
main          ← stable releases / gate-passed milestones only
  └── develop ← integration branch (default target for PRs)
        ├── feature/*   ← new features
        ├── bugfix/*    ← bug fixes
        ├── docs/*      ← documentation changes
        └── infra/*     ← CI/CD, tooling, infrastructure
```

### Branch Rules

| Branch     | Purpose                              | Merge Target | Protection           |
|------------|--------------------------------------|--------------|----------------------|
| `main`     | Stable, gate-passed milestones       | —            | Require CI pass, no direct push |
| `develop`  | Integration of completed features    | `main`       | Require CI pass      |
| `feature/*`| New functionality                    | `develop`    | —                    |
| `bugfix/*` | Bug fixes                            | `develop`    | —                    |
| `docs/*`   | Documentation updates                | `develop`    | —                    |
| `infra/*`  | CI/CD and tooling changes            | `develop`    | —                    |

### Workflow

1. **Create branch** from `develop`:
   ```bash
   git checkout develop
   git pull origin develop
   git checkout -b feature/my-feature
   ```

2. **Commit** with conventional commit messages:
   ```
   feat: add Gazebo world file for indoor environment
   fix: correct IMU noise model parameters
   docs: update Phase 2 requirements
   ci: add firmware binary size tracking
   refactor: extract sensor fusion module
   test: add hover stability test case
   ```

3. **Push and open PR** against `develop`:
   ```bash
   git push -u origin feature/my-feature
   ```

4. **Self-review** using the PR checklist (see template below).

5. **Merge** after CI passes. Use squash merge for feature branches.

6. **Merge `develop` → `main`** only at stage gate milestones.

### Commit Message Format

```
<type>(<scope>): <short description>

<optional body>

<optional footer: Refs #issue, Closes #issue>
```

**Types:** `feat`, `fix`, `docs`, `ci`, `refactor`, `test`, `chore`, `perf`
**Scopes:** `sim`, `fw`, `rl`, `hw`, `safety`, `infra`, `docs`

## Code Style

### Python
- Formatter/linter: **Ruff** (configured in `pyproject.toml`)
- Type checking: **mypy** (relaxed mode for early development)
- Line length: 100 characters

### C/C++ (Firmware)
- Style: Google C++ Style Guide (adapted)
- Linter: **cppcheck** + **clang-tidy**
- Naming: `snake_case` for functions/variables, `UPPER_CASE` for constants

### Documentation
- All user-facing docs in Markdown (MkDocs-compatible)
- Architecture diagrams in Mermaid (`.mmd` files)
- API docs auto-generated from docstrings

## Pre-commit Hooks

Pre-commit hooks are configured in `.pre-commit-config.yaml`. Install them:

```bash
pip install pre-commit
pre-commit install
```

Hooks run automatically on `git commit`. To run manually:

```bash
pre-commit run --all-files
```

## Pull Request Process

1. Fill out the PR template completely
2. Ensure CI passes (sim + firmware + docs pipelines)
3. Self-review using the checklist
4. Squash merge into `develop`

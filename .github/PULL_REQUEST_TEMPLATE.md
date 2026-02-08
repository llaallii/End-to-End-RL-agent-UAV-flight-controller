## Summary

<!-- Brief description of what this PR does -->

## Type of Change

- [ ] `feat`: New feature
- [ ] `fix`: Bug fix
- [ ] `docs`: Documentation only
- [ ] `ci`: CI/CD pipeline changes
- [ ] `refactor`: Code restructuring (no behavior change)
- [ ] `test`: Adding/updating tests
- [ ] `chore`: Maintenance (dependencies, configs)

## Scope

- [ ] Simulation (`simulation/`)
- [ ] Firmware (`firmware/`)
- [ ] RL Training (`training/`)
- [ ] Hardware (`hardware/`)
- [ ] Safety
- [ ] Infrastructure / CI
- [ ] Documentation (`docs/`)

## Related Requirements

<!-- Link to Doorstop requirement IDs this PR addresses, e.g., SIM-001, RL-007 -->

## Self-Review Checklist

### Code Quality
- [ ] Code follows the style guide (`ruff` / `cppcheck` pass)
- [ ] No new warnings introduced
- [ ] Type hints added for new Python functions
- [ ] Meaningful variable/function names used

### Testing
- [ ] New/updated tests cover the changes
- [ ] All existing tests still pass
- [ ] Edge cases considered and tested

### Documentation
- [ ] Code comments explain *why*, not *what*
- [ ] README or relevant docs updated if behavior changed
- [ ] Doorstop requirements updated if scope changed
- [ ] Architecture diagrams updated if structure changed

### Safety (if applicable)
- [ ] Safety-critical changes reviewed against SAF requirements
- [ ] Failure modes considered and handled
- [ ] No unbounded loops or dynamic allocation in real-time paths

## Test Results

<!-- Paste CI output or local test results -->

```
# pytest output or test summary
```

## Screenshots / Plots

<!-- If applicable: simulation screenshots, training curves, etc. -->

# RL Training

Reinforcement learning training pipeline using Stable-Baselines3 and Gymnasium.

## Structure

```
training/
├── envs/            # Custom Gymnasium environments
├── policies/        # Policy network architectures
├── configs/         # Hyperparameter configurations
├── scripts/         # Training & evaluation scripts
├── callbacks/       # SB3 training callbacks
├── logs/            # Training logs (gitignored)
└── models/          # Saved model checkpoints (gitignored)
```

## Frameworks

- [Stable-Baselines3](https://github.com/DLR-RM/stable-baselines3) — RL algorithms
- [Gymnasium](https://gymnasium.farama.org/) — Environment interface
- [Weights & Biases](https://wandb.ai/) — Experiment tracking (optional)

"""Training callbacks for SB3: evaluation, checkpointing, and metric logging."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from stable_baselines3.common.callbacks import (
    BaseCallback,
    CallbackList,
    CheckpointCallback,
    EvalCallback,
)
from stable_baselines3.common.vec_env import VecEnv


class MetricLoggerCallback(BaseCallback):
    """Log per-episode metrics to TensorBoard.

    Tracks episode reward, length, and final position error from the info dict
    provided by QuadrotorHoverEnv.
    """

    def __init__(self, verbose: int = 0) -> None:
        super().__init__(verbose)
        self._episode_rewards: list[float] = []
        self._episode_lengths: list[int] = []

    def _on_step(self) -> bool:
        # Check for episode completion in the info buffer
        infos = self.locals.get("infos", [])
        for info in infos:
            episode_info = info.get("episode")
            if episode_info is not None:
                ep_reward = float(episode_info["r"])
                ep_length = int(episode_info["l"])
                self._episode_rewards.append(ep_reward)
                self._episode_lengths.append(ep_length)

                self.logger.record("rollout/ep_rew_mean_custom", ep_reward)
                self.logger.record("rollout/ep_len_mean_custom", ep_length)

                # Log final position error if available
                final_pos = info.get("position")
                goal = info.get("goal")
                if final_pos is not None and goal is not None:
                    pos_error = float(np.linalg.norm(np.array(goal) - np.array(final_pos)))
                    self.logger.record("rollout/final_pos_error", pos_error)

        return True


def make_callbacks(
    eval_env: VecEnv,
    log_dir: str | Path,
    model_dir: str | Path,
    eval_freq: int = 10_000,
    n_eval_episodes: int = 5,
    checkpoint_freq: int = 50_000,
    verbose: int = 1,
) -> CallbackList:
    """Create the standard callback stack for hover training.

    Args:
        eval_env: Vectorized evaluation environment.
        log_dir: Directory for TensorBoard logs.
        model_dir: Directory for model checkpoints.
        eval_freq: Steps between evaluations.
        n_eval_episodes: Episodes per evaluation.
        checkpoint_freq: Steps between checkpoint saves.
        verbose: Verbosity level.

    Returns:
        Combined callback list.
    """
    log_dir = Path(log_dir)
    model_dir = Path(model_dir)

    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=str(model_dir / "best"),
        log_path=str(log_dir / "eval"),
        eval_freq=eval_freq,
        n_eval_episodes=n_eval_episodes,
        deterministic=True,
        verbose=verbose,
    )

    checkpoint_callback = CheckpointCallback(
        save_freq=checkpoint_freq,
        save_path=str(model_dir / "checkpoints"),
        name_prefix="hover_sac",
        verbose=verbose,
    )

    metric_callback = MetricLoggerCallback(verbose=verbose)

    return CallbackList([eval_callback, checkpoint_callback, metric_callback])

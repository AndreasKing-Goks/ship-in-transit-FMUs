from tensorboard.backend.event_processing import event_accumulator
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, ScalarFormatter, FuncFormatter

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

def get_tensorboard_event_files(run_dir):
    """
    Return all TensorBoard event files directly inside `run_dir`,
    sorted by their event-file timestamp.

    Parameters
    ----------
    run_dir : str | Path
        Directory containing events.out.tfevents.* files.

    Returns
    -------
    tuple[Path, ...]
        Sorted TensorBoard event-file paths.
    """
    run_dir = Path(run_dir)

    if not run_dir.is_dir():
        raise NotADirectoryError(
            f"TensorBoard run directory does not exist:\n{run_dir}"
        )

    def sort_key(event_file):
        # Expected structure:
        # events.out.tfevents.<timestamp>.<hostname>.<pid>.<index>
        parts = event_file.name.split(".")

        try:
            timestamp = int(parts[3])
        except (IndexError, ValueError):
            timestamp = int(event_file.stat().st_mtime)

        return timestamp, event_file.name

    event_files = tuple(
        sorted(
            run_dir.glob("events.out.tfevents.*"),
            key=sort_key,
        )
    )

    if not event_files:
        raise FileNotFoundError(
            f"No events.out.tfevents.* files found in:\n{run_dir}"
        )

    return event_files

def reduce_plot_density(plot_df, bin_size=50_000, agg="mean"):
    """
    Reduce plot density by grouping points into fixed step bins.

    Parameters
    ----------
    plot_df : pd.DataFrame
        Must contain columns: step, value.
    bin_size : int
        Step width of each bin. Example: 50_000 or 100_000.
    agg : str
        Aggregation method: "mean", "median", "max", "min".

    Returns
    -------
    pd.DataFrame
        Downsampled dataframe with one point per bin.
    """
    plot_df = plot_df.copy()
    plot_df["step_bin"] = (plot_df["step"] // bin_size) * bin_size

    if agg == "mean":
        reduced_df = plot_df.groupby("step_bin", as_index=False)["value"].mean()
    elif agg == "median":
        reduced_df = plot_df.groupby("step_bin", as_index=False)["value"].median()
    elif agg == "max":
        reduced_df = plot_df.groupby("step_bin", as_index=False)["value"].max()
    elif agg == "min":
        reduced_df = plot_df.groupby("step_bin", as_index=False)["value"].min()
    else:
        raise ValueError(f"Unknown aggregation method: {agg}")

    reduced_df = reduced_df.rename(columns={"step_bin": "step"})
    return reduced_df

def million_formatter(x, pos):
    if x == 0:
        return "0"
    return f"{x / 1_000_000:.0f}M"

# tb_run_dir = (
#     ROOT
#     / "EBASTv2_train"
#     / "trained_model"
#     / "EB-ASTv2_train_ppo_2026-07-26_21-54-50_3ace"
#     / "tb"
#     / "EB-ASTv2_train_ppo_0"
# )

tb_run_dir = (
    ROOT
    / "EBASTv2_train"
    / "trained_model"
    / "EB-ASTv2_train_rppo_2026-07-26_21-08-37_330c"
    / "tb"
    / "EB-ASTv2_train_rppo_0"
)

# tb_run_dir = (
#     ROOT
#     / "EBASTv2_train"
#     / "trained_model"
#     / "EB-ASTv2_train_sac_2026-07-26_21-08-37_acea"
#     / "tb"
#     / "EB-ASTv2_train_sac_0"
# )

tb_runs = get_tensorboard_event_files(tb_run_dir)

# PPO and RPPO
# Available tags:
#'rollout/ep_len_mean'
#'rollout/ep_rew_mean'
#'time/fps'
#'train/approx_kl'
#'train/clip_fraction'
#'train/clip_range'
#'train/entropy_loss'
#'train/explained_variance'
#'train/learning_rate'
#'train/loss'
#'train/policy_gradient_loss'
#'train/std'
#'train/value_loss'

# SAC
# Available tags:
#'rollout/ep_len_mean' 
#'rollout/ep_rew_mean'
#'train/actor_loss'
#'train/critic_loss'
#'train/ent_coef'
#'train/ent_coef_loss'
#'train/learning_rate'
#'train/n_updates'

tags_to_plot = ['rollout/ep_len_mean', 'rollout/ep_rew_mean', 'train/loss', 'train/policy_gradient_loss', 'train/value_loss']   # PPO and RPPO
# tags_to_plot = ['rollout/ep_len_mean', 'rollout/ep_rew_mean', 'train/actor_loss', 'train/critic_loss', 'train/learning_rate']   # SAC

all_rows = []

for tag in tags_to_plot:
    step_offset = 0

    for run_idx, run_dir in enumerate(tb_runs):
        ea = event_accumulator.EventAccumulator(str(run_dir))
        ea.Reload()

        events = ea.Scalars(tag)

        if not events:
            continue

        min_step = min(e.step for e in events)
        max_step = max(e.step for e in events)

        for e in events:
            all_rows.append({
                "run_idx": run_idx,
                "tag": tag,
                "step_original": e.step,
                "step": (e.step - min_step) + step_offset,
                "value": e.value,
                "wall_time": e.wall_time,
            })

        step_offset += (max_step - min_step) + 1

df = pd.DataFrame(all_rows)

for tag in tags_to_plot:
    plot_df = df[df["tag"] == tag].sort_values("step")

    # # Reduce SAC plot density
    # # Use 50_000 or 100_000 depending on how smooth you want it.
    # plot_df = reduce_plot_density(
    #     plot_df,
    #     bin_size=50_000,
    #     agg="mean",
    # )

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(
        plot_df["step"],
        plot_df["value"],
        linewidth=1.2,
        label=tag,
    )

    ax.set_xlim(left=0)
    ax.xaxis.set_major_locator(MultipleLocator(1_000_000))
    ax.xaxis.set_major_formatter(FuncFormatter(million_formatter))

    ax.set_xlabel("Step (Millions)")
    ax.set_ylabel("Value")

    ax.legend(loc="lower right", frameon=False)

    ax.grid(True, alpha=0.3)
    ax.tick_params(axis="x", labelsize=9)
    ax.tick_params(axis="y", labelsize=9)

    fig.tight_layout()

plt.show()
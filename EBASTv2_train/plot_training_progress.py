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

# Single plot
single_plot = False
# single_plot = True

if single_plot:
    
    # tb_run_dir = (
    #     ROOT
    #     / "EBASTv2_train"
    #     / "trained_model"
    #     / "EB-ASTv2_train_ppo_2026-07-26_21-54-50_3ace"
    #     / "tb"
    #     / "EB-ASTv2_train_ppo_0"
    # )

    # tb_run_dir = (
    #     ROOT
    #     / "EBASTv2_train"
    #     / "trained_model"
    #     / "EB-ASTv2_train_rppo_2026-07-26_21-08-37_330c"
    #     / "tb"
    #     / "EB-ASTv2_train_rppo_0"
    # )

    tb_run_dir = (
        ROOT
        / "EBASTv2_train"
        / "trained_model"
        / "EB-ASTv2_train_sac_2026-08-04_19-10-27_c531"
        / "tb"
        / "EB-ASTv2_train_sac_0"
    )

    tb_runs = get_tensorboard_event_files(tb_run_dir)

    # tags_to_plot = ['rollout/ep_len_mean', 'rollout/ep_rew_mean', 'train/loss', 'train/policy_gradient_loss', 'train/value_loss']   # PPO and RPPO
    tags_to_plot = ['rollout/ep_len_mean', 'rollout/ep_rew_mean', 'train/actor_loss', 'train/critic_loss', 'train/learning_rate']   # SAC

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

        # Reduce SAC plot density
        # Use 50_000 or 100_000 depending on how smooth you want it.
        plot_df = reduce_plot_density(
            plot_df,
            bin_size=50_000,
            agg="mean",
        )

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
    
multi_plots = True
# multi_plots = False

if multi_plots:

    tags_to_plot = [
        "rollout/ep_len_mean",
        "rollout/ep_rew_mean",
    ]

    # ---------------------------------------------------------
    # TensorBoard directories
    # ---------------------------------------------------------
    tb_run_dir_rppo = (
        ROOT
        / "EBASTv2_train"
        / "trained_model"
        / "EB-ASTv2_train_rppo_2026-07-26_21-08-37_330c"
        / "tb"
        / "EB-ASTv2_train_rppo_0"
    )

    tb_run_dir_sac = (
        ROOT
        / "EBASTv2_train"
        / "trained_model"
        / "EB-ASTv2_train_sac_2026-08-04_19-10-27_c531"
        / "tb"
        / "EB-ASTv2_train_sac_0"
    )

    tb_run_dir_ppo = (
        ROOT
        / "EBASTv2_train"
        / "trained_model"
        / "EB-ASTv2_train_ppo_2026-07-26_21-54-50_3ace"
        / "tb"
        / "EB-ASTv2_train_ppo_0"
    )

    # ---------------------------------------------------------
    # Methods
    # ---------------------------------------------------------
    tb_run_dirs_and_params = {
        "rppo": [tb_run_dir_rppo, "b"],
        "sac": [tb_run_dir_sac, "r"],
        # "ppo":  [tb_run_dir_ppo, "g"],
    }

    # ---------------------------------------------------------
    # Output directory
    # ---------------------------------------------------------
    save_dir = (
        ROOT
        / "EBASTv2_train"
        / "simulated_trained_model"
        / "plots_for_paper"
    )

    save_dir.mkdir(parents=True, exist_ok=True)

    # ---------------------------------------------------------
    # Plot-specific names
    # ---------------------------------------------------------
    plot_settings = {
        "rollout/ep_rew_mean": {
            "filename": "rppo_v_sac_reward.pdf",
            "ylabel": "Mean episode reward",
        },
        "rollout/ep_len_mean": {
            "filename": "rppo_v_sac_eps_len.pdf",
            "ylabel": "Mean episode length",
        },
    }

    # ---------------------------------------------------------
    # Read TensorBoard data
    # ---------------------------------------------------------
    all_rows = []

    for method in tb_run_dirs_and_params.keys():

        for tag in tags_to_plot:

            step_offset = 0

            tb_runs = get_tensorboard_event_files(
                tb_run_dirs_and_params[method][0]
            )

            for run_idx, run_dir in enumerate(tb_runs):

                ea = event_accumulator.EventAccumulator(
                    str(run_dir)
                )
                ea.Reload()

                events = ea.Scalars(tag)

                if not events:
                    continue

                min_step = min(e.step for e in events)
                max_step = max(e.step for e in events)

                for e in events:

                    all_rows.append({
                        "method": method,
                        "run_idx": run_idx,
                        "tag": tag,
                        "step_original": e.step,
                        "step": (
                            e.step - min_step
                        ) + step_offset,
                        "value": e.value,
                        "wall_time": e.wall_time,
                    })

                step_offset += (
                    max_step - min_step
                ) + 1

    df_all = pd.DataFrame(all_rows)

    # ---------------------------------------------------------
    # Bin size
    # ---------------------------------------------------------
    BIN_SIZE = 50_000

    # ---------------------------------------------------------
    # Create one figure per TensorBoard tag
    # ---------------------------------------------------------
    for tag in tags_to_plot:

        # Sized for two figures side-by-side in the thesis
        fig, ax = plt.subplots(
            figsize=(3.4, 1.9),
            dpi=150,
        )

        for method in tb_run_dirs_and_params.keys():

            plot_df = df_all[
                (df_all["method"] == method)
                & (df_all["tag"] == tag)
            ].sort_values("step").copy()

            if plot_df.empty:
                continue

            # -------------------------------------------------
            # Assign observations to training-step bins
            # -------------------------------------------------
            plot_df["bin"] = (
                plot_df["step"] // BIN_SIZE
            ) * BIN_SIZE

            # -------------------------------------------------
            # Mean and interquartile range
            # -------------------------------------------------
            stats = (
                plot_df.groupby("bin")["value"]
                .agg(
                    mean="mean",
                    q25=lambda x: x.quantile(0.0), #(0.25),
                    q75=lambda x: x.quantile(1.0), #(0.75),
                )
                .reset_index()
            )

            color = tb_run_dirs_and_params[method][1]

            # Mean
            ax.plot(
                stats["bin"],
                stats["mean"],
                linewidth=1.1,
                label=method.upper(),
                color=color,
            )

            # Interquartile range
            ax.fill_between(
                stats["bin"],
                stats["q25"],
                stats["q75"],
                color=color,
                alpha=0.18,
                linewidth=0,
            )

        # -----------------------------------------------------
        # Axes
        # -----------------------------------------------------
        ax.set_xlim(left=0)

        ax.xaxis.set_major_locator(
            MultipleLocator(1_000_000)
        )

        ax.xaxis.set_major_formatter(
            FuncFormatter(million_formatter)
        )

        ax.set_xlabel(
            "Training steps (millions)",
            fontsize=7.5,
            labelpad=1,
        )

        ax.set_ylabel(
            plot_settings[tag]["ylabel"],
            fontsize=7.5,
            labelpad=1,
        )

        # -----------------------------------------------------
        # Ticks
        # -----------------------------------------------------
        ax.tick_params(
            axis="both",
            which="major",
            labelsize=6.5,
            direction="in",
            length=2.5,
            pad=1,
        )

        # -----------------------------------------------------
        # Legend
        # -----------------------------------------------------
        ax.legend(
            loc="best",
            frameon=False,
            fontsize=6.0,
            handlelength=1.5,
            handletextpad=0.4,
            labelspacing=0.25,
        )

        # -----------------------------------------------------
        # Grid
        # -----------------------------------------------------
        ax.grid(
            True,
            linestyle=":",
            linewidth=0.45,
            alpha=0.4,
        )

        # Cleaner paper-style axes
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

        ax.spines["left"].set_linewidth(0.7)
        ax.spines["bottom"].set_linewidth(0.7)

        # -----------------------------------------------------
        # Compact layout
        # -----------------------------------------------------
        fig.tight_layout(
            pad=0.15
        )

        # -----------------------------------------------------
        # Save as vector PDF
        # -----------------------------------------------------
        save_path = (
            save_dir
            / plot_settings[tag]["filename"]
        )

        fig.savefig(
            save_path,
            format="pdf",
            bbox_inches="tight",
            pad_inches=0.01,
        )

        print(f"Saved: {save_path}")

    plt.show()
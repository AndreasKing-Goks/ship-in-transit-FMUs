from tensorboard.backend.event_processing import event_accumulator
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, ScalarFormatter, FuncFormatter
from matplotlib.lines import Line2D

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

    # tb_run_dir = (
    #     ROOT
    #     / "EBASTv2_train"
    #     / "trained_model"
    #     / "EB-ASTv2_train_sac_2026-08-04_19-10-27_c531"
    #     / "tb"
    #     / "EB-ASTv2_train_sac_0"
    # )
    
    tb_run_dir = (
        ROOT
        / "EBASTv2_train"
        / "trained_model"
        / "join_results"
        / "EB-ASTv2_train_rppo_2026-08-22_04-51-03_4942_multi_configs_continue"
    )

    tb_runs = get_tensorboard_event_files(tb_run_dir)

    # tags_to_plot = ['rollout/ep_len_mean', 'rollout/ep_rew_mean', 'train/loss', 'train/policy_gradient_loss', 'train/value_loss']   # PPO and RPPO
    # tags_to_plot = ['rollout/ep_len_mean', 'rollout/ep_rew_mean', 'train/actor_loss', 'train/critic_loss', 'train/learning_rate']   # SAC
    
    tags_to_plot = ['rollout/ep_len_mean', 'rollout/ep_rew_mean']   # PPO and RPPO

    # ---------------------------------------------------------
    # Output directory
    # ---------------------------------------------------------
    save_dir = (
        ROOT
        / "EBASTv2_train"
        / "trained_model"
        / "join_results"
        / "EB-ASTv2_train_rppo_2026-08-22_04-51-03_4942_continue"
    )

    save_dir.mkdir(parents=True, exist_ok=True)

    # ---------------------------------------------------------
    # Plot-specific names
    # ---------------------------------------------------------
    plot_settings = {
        "rollout/ep_rew_mean": {
            "filename": "rppo_multi_configs_continue_reward.pdf",
            "ylabel": "Mean episode reward",
        },
        "rollout/ep_len_mean": {
            "filename": "rppo_multi_configs_continue_eps_len.pdf",
            "ylabel": "Mean episode length",
        },
    }
    
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

        # Sized for two figures side-by-side in the thesis
        fig, ax = plt.subplots(
            figsize=(3.4, 1.9),
            dpi=150,
        )
        ax.plot(
            plot_df["step"],
            plot_df["value"],
            linewidth=1.2,
            label=tag,
        )
        
        # Mark previous training region
        transition_step = 10_000_000

        # Light gray shading before 10M
        ax.axvspan(
            plot_df["step"].min(),
            transition_step,
            color="lightgray",
            alpha=0.35,
            zorder=0,
        )

        # Dashed vertical line at 10M
        ax.axvline(
            transition_step,
            color="gray",
            linestyle="--",
            linewidth=1.0,
            zorder=2,
        )
        
        # Do ordinary least-squares linear regression on the remaining data after 10M steps
        post_df = plot_df[plot_df["step"] >= transition_step]

        # Work in millions of steps so the slope is interpretable
        x_post = post_df["step"].to_numpy() / 1e6
        y_post = post_df["value"].to_numpy()

        slope, intercept = np.polyfit(x_post, y_post, 1)

        x_fit = np.array([x_post.min(), x_post.max()])
        y_fit = slope * x_fit + intercept

        ax.plot(
            x_fit * 1e6,
            y_fit,
            linestyle="--",
            linewidth=1.8,
            label=f"Post-retraining trend ({slope:+.2f}/M steps)",
        )
        
        # coeffs = np.polyfit(x_post, y_post, 2)
        # a, b, c = coeffs

        # x_fit = np.linspace(x_post.min(), x_post.max(), 200)
        # y_fit = a * x_fit**2 + b * x_fit + c

        # ax.plot(
        #     x_fit * 1e6,
        #     y_fit,
        #     linestyle="--",
        #     linewidth=1.8,
        #     label="Post-retraining quadratic fit",
        # )
        
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
            labelsize=5.5,
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
    
multi_plots = False
# multi_plots = True

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
    
test_score_plot = False
# test_score_plot = True

if test_score_plot:
    def plot_testing_scores(
        scores,
        save_path=None,
        xlabel="Testing score (%)",
    ):
        """
        Plot performance variability across independent runs.

        Individual runs are shown as scatter points.
        The diamond marker represents the mean.
        Horizontal error bars represent ±1 sample standard deviation.

        Parameters
        ----------
        scores : dict
            Dictionary containing testing scores in percentage units.

            Example:
            {
                "RPPO": [68, 64, 65, 68, 69],
                "PPO":  [58, 68, 66, 63, 71],
                "SAC":  [54, 57, 51, 43, 52],
            }

        save_path : str | Path | None
            Optional output path for saving the figure as PDF.

        xlabel : str
            Label of the x-axis.

        Returns
        -------
        pd.DataFrame
            Summary statistics for each method.
        """

        methods = ["RPPO", "PPO", "SAC"]
        
        colors  = {"RPPO": "blue", "PPO": "green", "SAC": "red"}

        # ---------------------------------------------------------
        # Check input
        # ---------------------------------------------------------
        for method in methods:

            if method not in scores:
                raise ValueError(
                    f"Missing testing scores for {method}"
                )

            if len(scores[method]) == 0:
                raise ValueError(
                    f"No testing scores provided for {method}"
                )

        # ---------------------------------------------------------
        # Figure
        # Wide and short for horizontal comparison
        # ---------------------------------------------------------
        fig, ax = plt.subplots(
            figsize=(6.8, 2.2),
            dpi=150,
        )

        summary_rows = []

        # ---------------------------------------------------------
        # Plot each method
        # ---------------------------------------------------------
        for y, method in enumerate(methods):

            values = np.asarray(
                scores[method],
                dtype=float,
            )

            # -----------------------------------------------------
            # Statistics
            # -----------------------------------------------------
            mean = np.mean(values)

            # Sample standard deviation
            if len(values) > 1:
                std = np.std(
                    values,
                    ddof=1,
                )
            else:
                std = 0.0

            median = np.median(values)

            q25 = np.percentile(
                values,
                25,
            )

            q75 = np.percentile(
                values,
                75,
            )

            summary_rows.append({
                "method": method,
                "n": len(values),
                "mean": mean,
                "std": std,
                "median": median,
                "q25": q25,
                "q75": q75,
                "min": np.min(values),
                "max": np.max(values),
            })

            # -----------------------------------------------------
            # Deterministic vertical jitter
            #
            # This only separates the individual observations
            # visually. The vertical displacement has no
            # statistical meaning.
            # -----------------------------------------------------
            jitter = np.linspace(
                -0.08,
                0.08,
                len(values),
            )

            # -----------------------------------------------------
            # Individual runs
            # -----------------------------------------------------
            scatter = ax.scatter(
                values,
                y + jitter,
                s=32,
                alpha=0.85,
                linewidth=0.6,
                zorder=3,
                color=colors[method]
            )

            # Use the automatically assigned scatter color
            color = scatter.get_facecolor()[0]

            # -----------------------------------------------------
            # Mean ± standard deviation
            # -----------------------------------------------------
            ax.errorbar(
                mean,
                y,
                xerr=std,
                fmt="D",
                markersize=6.0,
                capsize=4,
                capthick=1.2,
                elinewidth=1.3,
                linewidth=1.2,
                color=colors[method],
                zorder=4,
            )

        # ---------------------------------------------------------
        # X-axis
        # ---------------------------------------------------------
        ax.set_xlim(
            40,
            75,
        )

        ax.xaxis.set_major_locator(
            MultipleLocator(5)
        )

        ax.xaxis.set_major_formatter(
            FuncFormatter(
                lambda x, pos: f"{x:.0f}%"
            )
        )

        ax.set_xlabel(
            xlabel,
            fontsize=10,
            labelpad=2,
        )

        # ---------------------------------------------------------
        # Y-axis
        # ---------------------------------------------------------
        ax.set_ylim(
            -0.4,
            len(methods) - 0.6,
        )

        ax.set_yticks(
            np.arange(len(methods))
        )

        ax.set_yticklabels(
            methods
        )

        # RPPO at top, then PPO, then SAC
        ax.invert_yaxis()

        # No need for "Method" label because the method names
        # themselves make the axis clear.
        ax.set_ylabel("")

        # ---------------------------------------------------------
        # Ticks
        # ---------------------------------------------------------
        ax.tick_params(
            axis="both",
            which="major",
            labelsize=9,
            direction="in",
            length=3,
            width=0.8,
            pad=2,
        )

        # ---------------------------------------------------------
        # Grid
        #
        # Only vertical grid lines because score is now on x-axis
        # ---------------------------------------------------------
        ax.grid(
            True,
            axis="x",
            linestyle=":",
            linewidth=0.5,
            alpha=0.3,
        )

        # ---------------------------------------------------------
        # Cleaner paper-style axes
        # ---------------------------------------------------------
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

        ax.spines["left"].set_linewidth(0.9)
        ax.spines["bottom"].set_linewidth(0.9)

        # ---------------------------------------------------------
        # Compact layout
        # ---------------------------------------------------------
        fig.tight_layout(
            pad=0.3
        )

        # ---------------------------------------------------------
        # Save
        # ---------------------------------------------------------
        if save_path is not None:

            save_path = Path(
                save_path
            )

            save_path.parent.mkdir(
                parents=True,
                exist_ok=True,
            )

            fig.savefig(
                save_path,
                format="pdf",
                bbox_inches="tight",
                pad_inches=0.01,
            )

            print(
                f"Saved: {save_path}"
            )

        plt.show()

        # ---------------------------------------------------------
        # Summary statistics
        # ---------------------------------------------------------
        summary_df = pd.DataFrame(
            summary_rows
        )

        return summary_df
    
    test_scores = {
        "RPPO": [68, 64, 65, 68, 69],
        "PPO":  [58, 68, 66, 63, 71], # [58, 68, 71, 66, 63]
        "SAC":  [54, 57, 51, 43, 54],
    }
    
    save_dir = (
        ROOT
        / "EBASTv2_train"
        / "simulated_trained_model"
        / "plots_for_paper"
    )
    
    save_path = (
            save_dir
            / "score_variability_tests.pdf"
        )

    summary_df = plot_testing_scores(save_path=save_path, scores=test_scores)
    
    print(summary_df)


test_eps_len_plot = False
test_eps_len_plot = True

if test_eps_len_plot:
    def plot_test_eps_len(
        eps_lens,
        save_path=None,
        xlabel="Episode Length",
    ):
        """
        Plot episode_length variability across independent runs.

        Individual runs are shown as scatter points.
        The diamond marker represents the mean.
        Horizontal error bars represent ±1 sample standard deviation.

        Parameters
        ----------
        scores : dict
            Dictionary containing testing episode length in percentage units.

        save_path : str | Path | None
            Optional output path for saving the figure as PDF.

        xlabel : str
            Label of the x-axis.

        Returns
        -------
        pd.DataFrame
            Summary statistics for each method.
        """

        methods = ["RPPO", "PPO", "SAC"]
        data_tags = ["collision", "no_collision"]
        
        colors  = {"RPPO": "blue", "PPO": "green", "SAC": "red"}
        markers  = {"collision": "o", "no_collision": "X"}
        
        tag_offset = {
                    "collision": 0, #-0.1,
                    "no_collision": 0, #0.1,
                }

        # ---------------------------------------------------------
        # Check input
        # ---------------------------------------------------------
        for method in methods:

            if method not in eps_lens:
                raise ValueError(
                    f"Missing testing episode length for {method}"
                )

        # ---------------------------------------------------------
        # Figure
        # Wide and short for horizontal comparison
        # ---------------------------------------------------------
        fig, ax = plt.subplots(
            figsize=(6.8, 2.2),
            dpi=150,
        )

        summary_rows = []

        # ---------------------------------------------------------
        # Plot each method
        # ---------------------------------------------------------
        for y, method in enumerate(methods):
            for tag in data_tags:

                values = np.asarray(
                    eps_lens[method][tag],
                    dtype=float,
                )

                # -----------------------------------------------------
                # Statistics
                # -----------------------------------------------------
                mean = np.mean(values)

                # Sample standard deviation
                if len(values) > 1:
                    std = np.std(
                        values,
                        ddof=1,
                    )
                else:
                    std = 0.0

                median = np.median(values)

                q25 = np.percentile(
                    values,
                    25,
                )

                q75 = np.percentile(
                    values,
                    75,
                )

                summary_rows.append({
                    "method": method,
                    "tag": tag,
                    "n": len(values),
                    "mean": mean,
                    "std": std,
                    "median": median,
                    "q25": q25,
                    "q75": q75,
                    "min": np.min(values),
                    "max": np.max(values),
                })

                # -----------------------------------------------------
                # Deterministic vertical jitter
                #
                # This only separates the individual observations
                # visually. The vertical displacement has no
                # statistical meaning.
                # -----------------------------------------------------
                jitter = np.linspace(
                    -0.08,
                    0.08,
                    len(values),
                )
                
                # -----------------------------------------------------
                # Individual runs
                # -----------------------------------------------------
                scatter = ax.scatter(
                    values,
                    y + jitter + tag_offset[tag],
                    s=32,
                    alpha=0.85,
                    linewidth=0.6,
                    zorder=3,
                    color=colors[method],
                    marker=markers[tag]
                )

                # Use the automatically assigned scatter color
                color = scatter.get_facecolor()[0]

                # -----------------------------------------------------
                # Mean ± standard deviation
                # -----------------------------------------------------
                ax.errorbar(
                    mean,
                    y + tag_offset[tag],
                    xerr=std,
                    fmt="D",
                    markersize=6.0,
                    capsize=4,
                    capthick=1.2,
                    elinewidth=1.3,
                    linewidth=1.2,
                    color=colors[method],
                    zorder=4,
                )

        # ---------------------------------------------------------
        # X-axis
        # ---------------------------------------------------------
        ax.set_xlim(
            5.0,
            8.0,
        )

        # ax.xaxis.set_major_locator(
        #     MultipleLocator(1)
        # )

        # ax.xaxis.set_major_formatter(
        #     FuncFormatter(
        #         lambda x, pos: f"{x:.0f}%"
        #     )
        # )

        ax.set_xlabel(
            xlabel,
            fontsize=10,
            labelpad=2,
        )

        # ---------------------------------------------------------
        # Y-axis
        # ---------------------------------------------------------
        ax.set_ylim(
            -0.4,
            len(methods) - 0.6,
        )

        ax.set_yticks(
            np.arange(len(methods))
        )

        ax.set_yticklabels(
            methods
        )

        # RPPO at top, then PPO, then SAC
        ax.invert_yaxis()

        # No need for "Method" label because the method names
        # themselves make the axis clear.
        ax.set_ylabel("")

        # ---------------------------------------------------------
        # Ticks
        # ---------------------------------------------------------
        ax.tick_params(
            axis="both",
            which="major",
            labelsize=9,
            direction="in",
            length=3,
            width=0.8,
            pad=2,
        )

        # ---------------------------------------------------------
        # Grid
        #
        # Only vertical grid lines because score is now on x-axis
        # ---------------------------------------------------------
        ax.grid(
            True,
            axis="x",
            linestyle=":",
            linewidth=0.5,
            alpha=0.3,
        )

        # ---------------------------------------------------------
        # Cleaner paper-style axes
        # ---------------------------------------------------------
        ax.spines["top"].set_visible(False)
        ax.spines["right"].set_visible(False)

        ax.spines["left"].set_linewidth(0.9)
        ax.spines["bottom"].set_linewidth(0.9)

        # ---------------------------------------------------------
        # Compact layout
        # ---------------------------------------------------------
        fig.tight_layout(
            pad=0.3
        )
        
        # ---------------------------------------------------------
        # Legend
        # ---------------------------------------------------------
        legend_handles = [
            Line2D(
                [0],
                [0],
                marker="o",
                linestyle="None",
                color="black",
                markersize=6,
                label="Collision",
            ),
            Line2D(
                [0],
                [0],
                marker="X",
                linestyle="None",
                color="black",
                markersize=6,
                label="No collision",
            ),
        ]

        ax.legend(
            handles=legend_handles,
            loc="upper center",
            bbox_to_anchor=(0.5, 1.18),
            ncol=2,
            frameon=False,
            fontsize=9,
        )

        # ---------------------------------------------------------
        # Save
        # ---------------------------------------------------------
        if save_path is not None:

            save_path = Path(
                save_path
            )

            save_path.parent.mkdir(
                parents=True,
                exist_ok=True,
            )

            fig.savefig(
                save_path,
                format="pdf",
                bbox_inches="tight",
                pad_inches=0.01,
            )

            print(
                f"Saved: {save_path}"
            )

        plt.show()

        # ---------------------------------------------------------
        # Summary statistics
        # ---------------------------------------------------------
        summary_df = pd.DataFrame(
            summary_rows
        )

        return summary_df
    
    eps_lens = {
        "RPPO": {
            "collision": [5.6, 5.6, 5.6, 5.7, 5.5], #[68, 64, 65, 68, 69],
            "no_collision": [6.6, 6.5, 6.8, 6.7, 6.6]
            },
        "PPO": {
            "collision": [5.3, 5.4, 5.5, 5.4, 5.2], #[58, 68, 66, 63, 71], 
            "no_collision":  [6.8, 6.6, 6.6, 6.7, 6.7]
            },
        "SAC": 
            {"collision": [6.9, 7.0, 7.0, 6.9, 7.1], #[54, 57, 51, 43, 54],
             "no_collision":  [7.9, 7.7, 7.7, 7.5, 7.6]
             }
    }
    
    save_dir = (
        ROOT
        / "EBASTv2_train"
        / "simulated_trained_model"
        / "plots_for_paper"
    )
    
    save_path = (
            save_dir
            / "episode_length_variability_tests.pdf"
        )

    summary_df = plot_test_eps_len(save_path=save_path, eps_lens=eps_lens)
    
    print(summary_df)
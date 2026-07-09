from pathlib import Path
from tensorboard.backend.event_processing import event_accumulator
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator, ScalarFormatter, FuncFormatter

def million_formatter(x, pos):
    if x == 0:
        return "0"
    return f"{x / 1_000_000:.0f}M"

tb_runs = (
    r"C:\Users\andre\0_PhD_Projects\ShipTransit_OptiStress\ship-in-transit-FMUs\EBASTv2_train\trained_model\EB-ASTv2_train_2ts_2026-06-13_22-30-22_69f2\tb\EB-ASTv2_train_2ts_1\events.out.tfevents.1781382632.idun-01-04.1032566.0",
    r"C:\Users\andre\0_PhD_Projects\ShipTransit_OptiStress\ship-in-transit-FMUs\EBASTv2_train\trained_model\EB-ASTv2_train_2ts_continue_03_2026-06-13_22-30-22_69f2\tb\EB-ASTv2_train_2ts_0\events.out.tfevents.1781616463.idun-08-02.1040222.0",
    r"C:\Users\andre\0_PhD_Projects\ShipTransit_OptiStress\ship-in-transit-FMUs\EBASTv2_train\trained_model\EB-ASTv2_train_2ts_continue_7a_continue_03_2026-06-13_22-30-22_69f2\tb\EB-ASTv2_train_2ts_0\events.out.tfevents.1781855061.idun-08-02.3489068.0",
    r"C:\Users\andre\0_PhD_Projects\ShipTransit_OptiStress\ship-in-transit-FMUs\EBASTv2_train\trained_model\EB-ASTv2_train_2ts_continue_7c_continue_7a_continue_03_2026-06-13_22-30-22_69f2\tb\EB-ASTv2_train_2ts_0\events.out.tfevents.1782291112.idun-01-03.1394672.0"
)

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

tags_to_plot = ['rollout/ep_len_mean', 'rollout/ep_rew_mean', 'train/loss', 'train/policy_gradient_loss', 'train/value_loss']

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

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(plot_df["step"], plot_df["value"], linewidth=1.2,
            label=tag)

    ax.set_xlim(left=0)
    ax.xaxis.set_major_locator(MultipleLocator(1_000_000))
    ax.xaxis.set_major_formatter(FuncFormatter(million_formatter))

    ax.set_xlabel("Step (Millions)")
    ax.set_ylabel("Value")
    # ax.set_title(tag)
    
    ax.legend(loc="lower right", frameon=False)

    ax.grid(True, alpha=0.3)
    ax.tick_params(axis="x", labelsize=9)
    ax.tick_params(axis="y", labelsize=9)

    fig.tight_layout()
plt.show()
"""
This module provides utilities class for the Co-simulation
"""

from pathlib import Path
import re
import datetime
import uuid
import json

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# =============================================================================================================
# Ship Parameters Compiler
# =============================================================================================================
def compile_ship_params(ship_cfg: dict) -> dict:
    route = ship_cfg["route"]
    north = route["north"]
    east  = route["east"]
    speed = route["speed"]

    if len(north) < 2 or len(east) < 2:
        raise ValueError("Route must have at least 2 points to compute initial yaw.")

    # Mission Manager params
    mm = dict(ship_cfg["fmu_params"].get("MISSION_MANAGER", {}))  # ra, max_inner_wp, etc.
    mm["wp_start_north"] = float(north[0])
    mm["wp_start_east"]  = float(east[0])
    mm["wp_end_north"]   = float(north[-1])
    mm["wp_end_east"]    = float(east[-1])
    mm["wp_end_speed"]   = float(speed[-1])

    # Intermediate waypoints: points 1..-2
    iw_north = north[1:-1]
    iw_east  = east[1:-1]
    iw_speed = speed[0:-1]

    # If you want to enforce max_inner_wp:
    max_inner_wp = int(len(iw_north))
    if len(iw_north) != len(iw_east):
        raise ValueError("Route north/east lengths mismatch.")
    mm["max_inner_wp"] = max_inner_wp

    if max_inner_wp > 0:
        for i, (n_i, e_i, s_i) in enumerate(zip(iw_north, iw_east, iw_speed), start=1):
            mm[f"wp_{i}_north"] = float(n_i)
            mm[f"wp_{i}_east"]  = float(e_i)
            mm[f"wp_{i}_speed"] = float(s_i)
    
    # Initial parameters (For the altered MISSION_MANAGER and SHIP_MODEL params)
    params = {
        "MISSION_MANAGER": mm
    }
    
    # Repopulate the unaltered parameters into params_dict
    params_name_list = list(ship_cfg["fmu_params"].keys())
    altered_params_name_list = list(params.keys())
    unaltered_params_name_list = [p for p in params_name_list if (p not in altered_params_name_list)]
        
    for param_name in unaltered_params_name_list:
        params[param_name] = dict(ship_cfg["fmu_params"][param_name])
        
    # set enable_colav flag
    if "COLAV" in params_name_list:
        ship_cfg["enable_colav"] = True
    else:
        ship_cfg["enable_colav"] = False
    
    # Pass-through params for other FMUs
    return params


# =============================================================================================================
# Ship Draw
# =============================================================================================================
class ShipDraw:
    ''' This class is used to calculate the coordinates of each
        corner of 100 meter long and 20 meter wide ship seen from above,
        and rotate and translate the coordinates according to
        the ship heading and position
    '''

    def __init__(self, l, b):
        self.l = l
        self.b = b

    def local_coords(self, scale = 1.0):
        ''' Here the ship is pointing along the local
            x-axis with its center of origin (midship)
            at the origin
            1 denotes the left back corner
            2 denotes the left starting point of bow curvatiure
            3 denotes the bow
            4 the right starting point of the bow curve
            5 the right back cornier
        '''
        x1, y1 = -self.l / 2, -self.b / 2
        x2, y2 = self.l / 4, -self.b / 2
        x3, y3 = self.l / 2, 0.0
        x4, y4 = self.l / 4, self.b / 2
        x5, y5 = -self.l / 2, self.b / 2

        x = np.array([x1, x2, x3, x4, x5, x1]) * scale
        y = np.array([y1, y2, y3, y4, y5, y1]) * scale
        return x, y

    def rotate_coords(self, x, y, psi):
        ''' Rotates the ship an angle psi
        '''
        x_t = np.cos(psi) * x - np.sin(psi) * y
        y_t = np.sin(psi) * x + np.cos(psi) * y
        return x_t, y_t

    def translate_coords(self, x_ned, y_ned, north, east):
        ''' Takes in coordinates of the corners of the ship (in the ned-frame)
            and translates them in the north and east direction according to
            "north" and "east"
        '''
        x_t = x_ned + north
        y_t = y_ned + east
        return x_t, y_t
    
    
# =============================================================================================================
# Get Path
# =============================================================================================================
def get_ship_route_path(ROOT, route_filename):
    return str(ROOT / "data" / "route" / route_filename)

def get_ship_route_path_from_group(ROOT, group=None, route_filename=None, pattern="*.txt"):
    """
    If route_filename is None -> return the folder Path.
    If route_filename is "*"... use pattern to list files.
    Otherwise return the full path to a single file.
    """
    if group is None:
        base = Path(ROOT) / "data" / "route" 
    else:
        base = Path(ROOT) / "data" / "route" / group
        
    if route_filename is None:
        return base
    if route_filename == "*":
        # list all matching files (natural sort by number in name)
        files = list(base.glob(pattern))
        def nkey(p: Path):
            parts = re.findall(r"\d+|\D+", p.stem)
            return [int(x) if x.isdigit() else x.lower() for x in parts]
        return sorted(files, key=nkey)
    return base / route_filename

def get_map_path(ROOT, map_filename):
    return str(ROOT / "data" / "map" / map_filename)

def get_saved_model_path(ROOT, saved_model_filename):
    return str(ROOT / "saved_model" / saved_model_filename)

def get_trained_model_path(ROOT, 
                           model_name :str):
    model_path = str(ROOT / "trained_model" / model_name / "model")
    log_path   = str(ROOT / "trained_model" / model_name / "log")
    
    return model_path, log_path

def get_saved_anim_path(ROOT, 
                        model_name :str):
    save_path = str(ROOT / "trained_model" / model_name / "saved_anim")
    
    return save_path

def get_rl_csv_path(ROOT,
                    model_name:str,
                    csv_name:str):
    save_path = str(ROOT / "trained_model" / model_name / "rl_csv" / csv_name)
    
    return save_path

def get_trained_model_and_log_path(ROOT: Path, model_name: str, unique: bool = True):
    """
    Generate model and log paths with unique timestamp (and short UUID) suffix.
    
    Example:
        model_path, log_path = get_trained_model_path(ROOT, "AST-train")
        
        # Returns something like:
        # model_path = ".../trained_model/AST-train_2025-11-09_18-25-03_ab12/model"
        # log_path   = ".../trained_model/AST-train_2025-11-09_18-25-03_ab12/log"
        # tb_path   = ".../trained_model/AST-train_2025-11-09_18-25-03_ab12/tb"
    """
    # Base directory
    base_dir = Path(ROOT) / "trained_model"

    # Create unique suffix (timestamp + short UUID)
    if unique:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        short_id = uuid.uuid4().hex[:4]  # short unique string
        model_name_unique = f"{model_name}_{timestamp}_{short_id}"
    else:
        model_name_unique = model_name

    # Full paths
    model_path = str(base_dir / model_name_unique / "model")
    log_path   = str(base_dir / model_name_unique / "log")
    tb_path   = str(base_dir / model_name_unique / "tb")

    return model_path, log_path, tb_path


# =============================================================================================================
# Visualize BO results 
# =============================================================================================================
def view_bo_results_table(results_json_path, save_csv=True, csv_output_path=None):
    """
    Load BO results JSON and display metrics (and trial parameter values) in a formatted table.
    
    Parameters
    ----------
    results_json_path : str or Path
        Path to the ax_results.json file from BO experiment.
    save_csv : bool
        If True, save the table as CSV.
    csv_output_path : str or Path, optional
        Path to save CSV. If None, saves to same directory as JSON with "_metrics.csv" suffix.
    
    Returns
    -------
    pd.DataFrame
        DataFrame containing trial metadata, selected parameter values, and metrics.
    
    Example
    -------
    >>> df = view_bo_results_table("test_run/open_sea_one_ts_ho_bo/ax_results.json")
    """
    results_json_path = Path(results_json_path)
    
    # Load the results
    with open(results_json_path, "r") as f:
        data = json.load(f)
    
    # Extract parameters and metrics from each trial
    metrics_list = []
    for trial in data["history"]:
        trial_params = trial.get("parameters", {})
        row = {
            "trial": trial["trial_index"],
            "type": trial["trial_type"],
            "status": trial["trial_status"],
        }
        # Prefix parameter columns to avoid accidental name collisions.
        row.update({f"param_{k}": v for k, v in trial_params.items()})
        row.update(trial["metrics"])
        metrics_list.append(row)
    
    # Create DataFrame
    df = pd.DataFrame(metrics_list)
    
    # Display
    print(df.to_string())
    
    # Optional: save as CSV
    if save_csv:
        if csv_output_path is None:
            csv_output_path = results_json_path.parent / f"{results_json_path.stem}_metrics.csv"
        df.to_csv(csv_output_path, index=False)
        print(f"\nAlso saved to {csv_output_path}")
    
    return df


# =============================================================================================================
# Bayesian Optimization Results Visualization
# =============================================================================================================
# Colors per generation phase used by the BO result plots.
_PHASE_COLORS = {
    "Sobol": "#888888",            # gray   – random exploration phase
    "BoTorch": "#1f77b4",          # blue   – Bayesian (model-driven) phase
    "Fallback_Sobol": "#9467bd",   # purple – BoTorch fell back to a Sobol draw
}
_INFEASIBLE_COLOR = "#d62728"      # red    – trials that could not be evaluated


def _bo_phase_of(type_str):
    """Map an Ax generation_method string to a coarse phase label."""
    t = str(type_str)
    if "BoTorch" in t:
        return "BoTorch"
    if "Fallback" in t:
        return "Fallback_Sobol"
    return "Sobol"


def load_bo_results(csv_path):
    """Load a BO ``*_metrics.csv`` and add helper columns used by the plots.

    Parameters
    ----------
    csv_path : str or Path
        Path to the metrics CSV produced by ``view_bo_results_table``.

    Returns
    -------
    pd.DataFrame
        The trials sorted by index, with ``phase`` and ``feasible`` columns added.
    """
    df = pd.read_csv(csv_path)
    df = df.sort_values("trial").reset_index(drop=True)
    df["phase"] = df["type"].map(_bo_phase_of)
    # A trial is a "real" evaluation only if Ax completed it successfully.
    df["feasible"] = df["status"].astype(str).str.upper() == "COMPLETED"
    return df


def plot_bo_convergence(df, ax):
    """Objective per trial + running best. Lower is better (more dangerous)."""
    feasible = df[df["feasible"]].copy()
    infeasible = df[~df["feasible"]].copy()

    # Scatter each feasible trial's objective, colored by phase.
    for phase, color in _PHASE_COLORS.items():
        sub = feasible[feasible["phase"] == phase]
        if not sub.empty:
            ax.scatter(sub["trial"], sub["objective"], s=45, color=color,
                       label=f"{phase} (evaluated)", zorder=3, edgecolors="white",
                       linewidths=0.5)

    # Running best = cumulative minimum over feasible trials (Ax minimizes).
    if not feasible.empty:
        running_best = feasible["objective"].cummin()
        ax.step(feasible["trial"], running_best, where="post", color="black",
                linewidth=2, label="running best", zorder=4)

    # Mark infeasible / failed trials along the top so they are visible
    # without distorting the y-axis (their objective is inf or a penalty).
    if not infeasible.empty and not feasible.empty:
        top = feasible["objective"].max()
        ax.scatter(infeasible["trial"], np.full(len(infeasible), top),
                   marker="x", s=60, color=_INFEASIBLE_COLOR,
                   label="infeasible (failed)", zorder=5)

    ax.set_xlabel("Trial index")
    ax.set_ylabel("Objective  (= -danger_score, lower = better)")
    ax.set_title("Objective per trial and running best")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8, loc="best")


def plot_bo_parameter_trajectories(df, param_cols):
    """One subplot per parameter: value vs trial index, colored by phase."""
    n = len(param_cols)
    ncols = 2
    nrows = int(np.ceil(n / ncols))
    fig, axes = plt.subplots(nrows, ncols, figsize=(12, 3 * nrows), squeeze=False)
    axes = axes.ravel()

    for i, col in enumerate(param_cols):
        ax = axes[i]
        # Connecting line shows the path the algorithm took.
        ax.plot(df["trial"], df[col], color="lightgray", linewidth=1, zorder=1)
        for phase, color in _PHASE_COLORS.items():
            sub = df[df["phase"] == phase]
            if not sub.empty:
                ax.scatter(sub["trial"], sub[col], s=30, color=color,
                           label=phase, zorder=3)
        # Mark infeasible trials in red so you see which regions failed.
        infeasible = df[~df["feasible"]]
        if not infeasible.empty:
            ax.scatter(infeasible["trial"], infeasible[col], marker="x", s=45,
                       color=_INFEASIBLE_COLOR, zorder=4, label="infeasible")
        ax.set_title(col.replace("param_", ""), fontsize=10)
        ax.set_xlabel("Trial")
        ax.set_ylabel("Value")
        ax.grid(True, alpha=0.3)

    # Hide any unused subplots.
    for j in range(n, len(axes)):
        axes[j].set_visible(False)

    # Single shared legend.
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, fontsize=9)
    fig.suptitle("Search-space movement: parameter value per trial", y=1.0)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    return fig


def plot_bo_parallel_coordinates(df, param_cols):
    """Parallel-coordinates of feasible trials, each line colored by objective.

    Every trial is a poly-line crossing all parameter axes. Darker/low-objective
    lines mark the more dangerous (better) regions the optimizer favored.
    """
    feasible = df[df["feasible"]].copy()
    fig, ax = plt.subplots(figsize=(12, 6))
    if feasible.empty or not param_cols:
        ax.text(0.5, 0.5, "No feasible trials to plot", ha="center", va="center")
        return fig

    # Normalize each parameter axis to [0, 1] for comparable scales.
    norm = feasible[param_cols].copy()
    for col in param_cols:
        lo, hi = feasible[col].min(), feasible[col].max()
        norm[col] = (feasible[col] - lo) / (hi - lo) if hi > lo else 0.5

    x = np.arange(len(param_cols))
    obj = feasible["objective"].to_numpy()
    cmap = plt.get_cmap("viridis")
    o_lo, o_hi = obj.min(), obj.max()
    denom = (o_hi - o_lo) if o_hi > o_lo else 1.0

    for _, row in norm.assign(_obj=obj).iterrows():
        color = cmap((row["_obj"] - o_lo) / denom)
        ax.plot(x, row[param_cols].to_numpy(dtype=float), color=color,
                alpha=0.7, linewidth=1.2)

    ax.set_xticks(x)
    ax.set_xticklabels([c.replace("param_", "") for c in param_cols],
                       rotation=30, ha="right", fontsize=9)
    ax.set_ylabel("Normalized parameter value [0, 1]")
    ax.set_title("Parallel coordinates of feasible trials (color = objective)")
    ax.grid(True, alpha=0.3, axis="x")

    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(vmin=o_lo, vmax=o_hi))
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=ax)
    cbar.set_label("objective (lower = more dangerous = better)")
    fig.tight_layout()
    return fig


def plot_bo_results(csv_path, save=True, show=True):
    """Generate the three BO result figures from a metrics CSV.

    Produces:
        1. Convergence  – objective per trial + running best (should trend down).
        2. Parameter trajectories – how each parameter moves across trials.
        3. Parallel coordinates – feasible trials colored by objective.

    Parameters
    ----------
    csv_path : str or Path
        Path to the ``*_metrics.csv`` file from a BO experiment.
    save : bool
        If True, write the three PNG figures next to the CSV.
    show : bool
        If True, open the figure windows (blocks until closed).
    """
    csv_path = Path(csv_path)
    df = load_bo_results(csv_path)
    param_cols = [c for c in df.columns if c.startswith("param_")]
    if not param_cols:
        raise ValueError(f"No 'param_*' columns found in {csv_path}")

    out_dir = csv_path.parent
    stem = csv_path.stem

    # 1) Convergence
    fig1, ax1 = plt.subplots(figsize=(10, 6))
    plot_bo_convergence(df, ax1)
    fig1.tight_layout()

    # 2) Per-parameter trajectories
    fig2 = plot_bo_parameter_trajectories(df, param_cols)

    # 3) Parallel coordinates
    fig3 = plot_bo_parallel_coordinates(df, param_cols)

    if save:
        fig1.savefig(out_dir / f"{stem}_convergence.png", dpi=150, bbox_inches="tight")
        fig2.savefig(out_dir / f"{stem}_param_trajectories.png", dpi=150, bbox_inches="tight")
        fig3.savefig(out_dir / f"{stem}_parallel_coords.png", dpi=150, bbox_inches="tight")
        print(f"Saved 3 figures next to {csv_path.name} in {out_dir}")

    if show:
        plt.show()


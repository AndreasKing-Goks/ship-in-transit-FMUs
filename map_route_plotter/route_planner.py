from pathlib import Path
import sys, os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

# ---------- PATH SETUP ----------
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from orchestrator.utils import get_ship_route_path, get_map_path
from map_route_plotter.prepare_map_route import get_gdf_from_gpkg

# ============ USER KNOBS ============
ROUTE_FILENAME = "of_route.txt"
PRINT_INIT_MSG = True
POINT_MARKER_SIZE = 20
LABEL_FONTSIZE = 7.5

# -------- Map detail toggles --------
SHOW_COAST        = False
SHOW_WATER        = False
SHOW_WATERWAYS    = False
SHOW_FERRY_ROUTES = True
SHOW_HARBOURS     = False
SHOW_BRIDGES      = False
SHOW_TSS          = False
SHOW_DOCKS        = False

# GPKG layers
GPKG_PATH           = get_map_path(ROOT, "oslo_fjord.gpkg")
FRAME_LAYER         = "frame_3857"
OCEAN_LAYER         = "ocean_3857"
LAND_LAYER          = "land_3857"
COAST_LAYER         = "coast_3857"
WATER_LAYER         = "water_3857"
WATERWAYS_LAYER     = "waterways_3857"
FERRY_ROUTES_LAYER  = "ferry_routes_3857"
HARBOURS_LAYER      = "harbours_3857"
BRIDGES_LAYER       = "bridges_3857"
TSS_LAYER           = "tss_3857"
DOCKS_LAYER         = "docks_3857"


def unique_save_path(base_path: Path) -> Path:
    """Return a unique path by adding _1, _2, ... if needed."""
    if not base_path.exists():
        return base_path
    stem = base_path.stem
    suffix = base_path.suffix
    parent = base_path.parent
    i = 1
    while True:
        candidate = parent / f"{stem}_{i}{suffix}"
        if not candidate.exists():
            return candidate
        i += 1


class RoutePicker:
    def __init__(self, ax, save_path: Path):
        self.ax = ax
        self.fig = ax.figure
        self.save_path = save_path
        self.points = []  # stored as (north, east) to match your save logic
        self.labels = []
        self.point_scatters = []

        (self.line,) = ax.plot([], [], lw=1, color="red", linestyle="--", zorder=20)

        self.cid_click = self.fig.canvas.mpl_connect("button_press_event", self._onclick)
        self.cid_key   = self.fig.canvas.mpl_connect("key_press_event", self._onkey)

        self.help = self.ax.annotate(
            "Left-click: add waypoint    |    Undo    Clear    Finish & Save    Cancel\n"
            "Tip: Use mouse scroll/drag to navigate.",
            xy=(0.01, 0.01), xycoords="axes fraction", va="bottom", ha="left",
            bbox=dict(boxstyle="round,pad=0.25", fc="white", alpha=0.9), fontsize=LABEL_FONTSIZE/2
        )

        self.start_anno = None
        self.goal_anno = None

        plt.subplots_adjust(bottom=0.12)
        bx = 0.12
        bw, bh, pad = 0.15, 0.05, 0.02
        self.b_undo_ax   = self.fig.add_axes([bx + 0*(bw+pad), 0.02, bw, bh])
        self.b_clear_ax  = self.fig.add_axes([bx + 1*(bw+pad), 0.02, bw, bh])
        self.b_finish_ax = self.fig.add_axes([bx + 2*(bw+pad), 0.02, bw, bh])
        self.b_cancel_ax = self.fig.add_axes([bx + 3*(bw+pad), 0.02, bw, bh])

        self.b_undo   = Button(self.b_undo_ax,   "Undo")
        self.b_clear  = Button(self.b_clear_ax,  "Clear")
        self.b_finish = Button(self.b_finish_ax, "Finish & Save")
        self.b_cancel = Button(self.b_cancel_ax, "Cancel")
        
        self.b_undo.label.set_fontsize(LABEL_FONTSIZE)
        self.b_clear.label.set_fontsize(LABEL_FONTSIZE)
        self.b_finish.label.set_fontsize(LABEL_FONTSIZE)
        self.b_cancel.label.set_fontsize(LABEL_FONTSIZE)

        self.b_undo.on_clicked(self._on_undo)
        self.b_clear.on_clicked(self._on_clear)
        self.b_finish.on_clicked(self._on_finish)
        self.b_cancel.on_clicked(self._on_cancel)

    def _onclick(self, event):
        if event.inaxes != self.ax or event.button != 1:
            return
        if event.xdata is None or event.ydata is None:
            return

        e, n = float(event.xdata), float(event.ydata)
        self.points.append((n, e))  # keep your existing convention

        sc = self.ax.scatter([e], [n], s=POINT_MARKER_SIZE, color="red",
                             edgecolor="white", linewidth=0.8, zorder=21)
        self.point_scatters.append(sc)

        idx = len(self.points)
        self.labels.append(
            self.ax.annotate(
                f"{idx}", (e, n), xytext=(0, 10), textcoords="offset points",
                fontsize=LABEL_FONTSIZE,
                bbox=dict(boxstyle="round,pad=0.2", fc="w", alpha=0.8),
                zorder=22,
            )
        )

        self._refresh_line_and_tags()
        self.fig.canvas.draw_idle()

    def _onkey(self, event):
        if event.key in ("enter", "return"):
            self._on_finish(None)
        elif event.key in ("backspace", "delete"):
            self._on_undo(None)
        elif event.key == "escape":
            self._on_cancel(None)

    def _on_undo(self, _):
        if not self.points:
            return
        self.points.pop()

        lbl = self.labels.pop()
        lbl.remove()

        sc = self.point_scatters.pop()
        sc.remove()

        self._refresh_line_and_tags()
        self.fig.canvas.draw_idle()

    def _on_clear(self, _):
        self.points.clear()

        for l in self.labels:
            l.remove()
        self.labels.clear()

        for sc in self.point_scatters:
            sc.remove()
        self.point_scatters.clear()

        self._refresh_line_and_tags()
        self.fig.canvas.draw_idle()

    def _on_cancel(self, _):
        self._disconnect()
        print("Route planner cancelled.")

    def _on_finish(self, _):
        if not self.points:
            print("No points to save.")
            return

        self._disconnect()
        self.save_path.parent.mkdir(parents=True, exist_ok=True)
        out_path = unique_save_path(self.save_path)

        arr = np.array(self.points, dtype=float)  # shape (k, 2) = [north, east]
        header = "#Y/North, X/East"
        np.savetxt(out_path, arr, fmt="%.3f %.3f", header=header, comments="")

        print(f"Saved {len(self.points)} waypoints to: {out_path}")

    def _disconnect(self):
        self.fig.canvas.mpl_disconnect(self.cid_click)
        self.fig.canvas.mpl_disconnect(self.cid_key)
        for b in (self.b_undo, self.b_clear, self.b_finish, self.b_cancel):
            b.ax.set_alpha(0.3)
            b.ax.set_facecolor("#f0f0f0")
        self.fig.canvas.draw_idle()

    def _refresh_line_and_tags(self):
        if self.points:
            ys, xs = zip(*self.points)  # points stored as (north, east)
        else:
            ys, xs = [], []

        self.line.set_data(xs, ys)

        for tag in (self.start_anno, self.goal_anno):
            if tag is not None:
                try:
                    tag.remove()
                except Exception:
                    pass

        self.start_anno = None
        self.goal_anno = None

        if self.points:
            n0, e0 = self.points[0]
            self.start_anno = self.ax.annotate(
                "START", (e0, n0), xytext=(5, -9), textcoords="offset points",
                fontsize=LABEL_FONTSIZE, bbox=dict(boxstyle="round,pad=0.2", fc="w", alpha=0.8),
                zorder=23,
            )

            nL, eL = self.points[-1]
            self.goal_anno = self.ax.annotate(
                "END", (eL, nL), xytext=(5, -9), textcoords="offset points",
                fontsize=LABEL_FONTSIZE, bbox=dict(boxstyle="round,pad=0.2", fc="w", alpha=0.8),
                zorder=23,
            )


def main():
    (
        frame_gdf, ocean_gdf, land_gdf, coast_gdf, water_gdf,
        waterways_gdf, ferry_routes_gdf, harbours_gdf, bridges_gdf, tss_gdf, docks_gdf
    ) = get_gdf_from_gpkg(
        GPKG_PATH,
        frame_layer=FRAME_LAYER,
        ocean_layer=OCEAN_LAYER,
        land_layer=LAND_LAYER,
        coast_layer=COAST_LAYER,
        water_layer=WATER_LAYER,
        waterways_layer=WATERWAYS_LAYER,
        ferry_routes_layer=FERRY_ROUTES_LAYER,
        harbours_layer=HARBOURS_LAYER,
        bridges_layer=BRIDGES_LAYER,
        tss_layer=TSS_LAYER,
        docks_layer=DOCKS_LAYER,
    )

    minx, miny, maxx, maxy = frame_gdf.total_bounds
    map_w = maxx - minx
    map_h = maxy - miny
    aspect = map_w / map_h

    fig_width = 5.5
    fig_height = fig_width / aspect
    fig, ax = plt.subplots(figsize=(fig_width, fig_height), dpi=180)

    # ----- Basemap -----
    if not ocean_gdf.empty:
        ocean_gdf.plot(ax=ax, facecolor="#cfe8f7", edgecolor="none", zorder=0)

    if not land_gdf.empty:
        land_gdf.plot(ax=ax, facecolor="#dfe6d5", edgecolor="#7a8a6a", linewidth=0.35, zorder=1)

    if SHOW_WATER and not water_gdf.empty:
        water_gdf.plot(ax=ax, facecolor="#b7dcef", edgecolor="none", zorder=2)

    if SHOW_COAST and not coast_gdf.empty:
        coast_gdf.plot(ax=ax, color="#4f6650", linewidth=0.45, zorder=3)

    # ----- Optional overlays -----
    if SHOW_WATERWAYS and not waterways_gdf.empty:
        waterways_gdf.plot(ax=ax, color="#7fb6d6", linewidth=0.6, alpha=0.9, zorder=4)

    if SHOW_FERRY_ROUTES and not ferry_routes_gdf.empty:
        ferry_routes_gdf.plot(ax=ax, color="#5d6fd3", linewidth=0.25, linestyle="--", alpha=0.5, zorder=5)

    if SHOW_TSS and not tss_gdf.empty:
        tss_gdf.plot(ax=ax, color="#9c6ade", linewidth=1.0, linestyle=":", alpha=0.9, zorder=5)

    if SHOW_BRIDGES and not bridges_gdf.empty:
        bridges_gdf.plot(ax=ax, color="#6b4f3a", linewidth=1.2, alpha=0.9, zorder=6)

    if SHOW_DOCKS and not docks_gdf.empty:
        docks_gdf.plot(ax=ax, facecolor="#d9c27a", edgecolor="#8d7b45", linewidth=0.4, alpha=0.9, zorder=6)

    if SHOW_HARBOURS and not harbours_gdf.empty:
        harbours_gdf.plot(ax=ax, color="#c85a5a", markersize=14, alpha=0.85, zorder=7)

    # ----- Axes -----
    ax.set_xlim(minx, maxx)
    ax.set_ylim(miny, maxy)
    ax.set_aspect("equal")

    ax.set_xlabel("East [m]").set_fontsize(LABEL_FONTSIZE)
    ax.set_ylabel("North [m]").set_fontsize(LABEL_FONTSIZE)

    ax.ticklabel_format(axis="y", style="sci", scilimits=(0, 0))
    ax.yaxis.get_offset_text().set_fontsize(LABEL_FONTSIZE)
    ax.ticklabel_format(axis="x", style="sci", scilimits=(0, 0))
    ax.xaxis.get_offset_text().set_fontsize(LABEL_FONTSIZE)
    ax.tick_params(axis="both", labelsize=LABEL_FONTSIZE)

    ax.grid(True, color="0.82", linestyle="--", linewidth=0.5, zorder=100)

    for spine in ax.spines.values():
        spine.set_linewidth(0.8)
        spine.set_color("0.35")

    ax.set_title("Route Planner • Click to add waypoints • Buttons below to Finish/Undo/Clear/Cancel", fontsize=LABEL_FONTSIZE)

    save_path = Path(get_ship_route_path(ROOT, ROUTE_FILENAME))
    RoutePicker(ax, save_path=save_path)

    plt.subplots_adjust(left=0.08, right=0.99, top=0.93)
    plt.show()


if __name__ == "__main__":
    main()
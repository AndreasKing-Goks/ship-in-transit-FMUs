from pathlib import Path
import sys, os, glob
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

# ---------- PATH SETUP ----------
# Route up one level
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from orchestrator.utils import get_ship_route_path, get_map_path
from prepare_map import get_gdf_from_gpkg

# ============ USER KNOBS ============
ROUTE_FILENAME = "of_route.txt"  # without directories; will save under data/route/
PRINT_INIT_MSG = True             # small debug prints
POINT_MARKER_SIZE = 160           # big for visibility
LABEL_FONTSIZE = 10

# GPKG layers
GPKG_PATH   = get_map_path(ROOT, "oslo_fjord.gpkg")
FRAME_LAYER = "frame_3857"
OCEAN_LAYER = "ocean_3857"
LAND_LAYER  = "land_3857"
COAST_LAYER = "coast_3857"   # optional
WATER_LAYER = "water_3857"   # optional

# -----------------------------------

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
        self.points = []  # list of (E, N)
        self.labels = []
        (self.line,) = ax.plot([], [], lw=2)
        self.cid_click = self.fig.canvas.mpl_connect('button_press_event', self._onclick)
        self.cid_key   = self.fig.canvas.mpl_connect('key_press_event', self._onkey)

        # Instruction overlay
        self.help = self.ax.annotate(
            "Left-click: add waypoint    |    Undo    Clear    Finish & Save    Cancel\n"
            "Tip: Use mouse scroll/drag to navigate.",
            xy=(0.01, 0.01), xycoords='axes fraction', va='bottom', ha='left',
            bbox=dict(boxstyle="round,pad=0.25", fc="white", alpha=0.9), fontsize=9
        )

        # START/GOAL annotations (dynamic)
        self.start_anno = None
        self.goal_anno = None

        # Buttons
        # place buttons in an empty strip at the bottom
        plt.subplots_adjust(bottom=0.12)  # make room for buttons
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
        self.points.append((n, e)) # flipped
        # big marker for visibility
        self.ax.scatter([e], [n], s=POINT_MARKER_SIZE)
        # numbered label
        idx = len(self.points)
        self.labels.append(self.ax.annotate(
            f"{idx}", (e, n), xytext=(6, 6), textcoords='offset points',
            fontsize=LABEL_FONTSIZE, bbox=dict(boxstyle="round,pad=0.2", fc="w", alpha=0.8)
        ))
        self._refresh_line_and_tags()
        self.fig.canvas.draw_idle()

    def _onkey(self, event):
        if event.key in ('enter', 'return'):
            self._on_finish(None)
        elif event.key in ('backspace', 'delete'):
            self._on_undo(None)
        elif event.key == 'escape':
            self._on_cancel(None)

    def _on_undo(self, _):
        if not self.points:
            return
        self.points.pop()
        lbl = self.labels.pop()
        lbl.remove()
        # Remove last scatter by full redraw (simple & robust)
        self._full_redraw_points()
        self._refresh_line_and_tags()
        self.fig.canvas.draw_idle()

    def _on_clear(self, _):
        self.points.clear()
        for l in self.labels:
            l.remove()
        self.labels.clear()
        self._full_redraw_points()
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
        # Ensure dir exists
        self.save_path.parent.mkdir(parents=True, exist_ok=True)
        out_path = unique_save_path(self.save_path)
        arr = np.array(self.points, dtype=float)  # shape (k,2) = [E, N]
        # Save as:
        # Y/East, X/North
        # <E> <N>
        header = "#Y/North, X/East"
        np.savetxt(out_path, arr, fmt="%.3f %.3f", header=header, comments='')
        print(f"Saved {len(self.points)} waypoints to: {out_path}")

    def _disconnect(self):
        self.fig.canvas.mpl_disconnect(self.cid_click)
        self.fig.canvas.mpl_disconnect(self.cid_key)
        # grey out buttons
        for b in (self.b_undo, self.b_clear, self.b_finish, self.b_cancel):
            b.ax.set_alpha(0.3)
            b.ax.set_facecolor("#f0f0f0")
        self.fig.canvas.draw_idle()

    def _refresh_line_and_tags(self):
        if self.points:
            xs, ys = zip(*self.points)
        else:
            xs, ys = [], []
        self.line.set_data(xs, ys)

        # Update START/GOAL tags
        for tag in (self.start_anno, self.goal_anno):
            if tag is not None:
                try: tag.remove()
                except: pass
        if self.points:
            e0, n0 = self.points[0]
            self.start_anno = self.ax.annotate(
                "START", (e0, n0), xytext=(10, -18), textcoords='offset points',
                fontsize=10, bbox=dict(boxstyle="round,pad=0.2", fc="w", alpha=0.8)
            )
            eL, nL = self.points[-1]
            self.goal_anno = self.ax.annotate(
                "GOAL", (eL, nL), xytext=(10, -18), textcoords='offset points',
                fontsize=10, bbox=dict(boxstyle="round,pad=0.2", fc="w", alpha=0.8)
            )

    def _full_redraw_points(self):
        # We keep the basemap; only clear and redraw the route layer (line+labels+scatters).
        # Easiest is to clear and re-plot using stored points.
        # Remove all artist collections created by our scatters by re-drawing:
        # (Avoid ax.cla() since it would remove the basemap.)
        # -> Simple approach: clear the line data and relabel by re-scatter.
        self.line.set_data([], [])
        # Remove all existing point markers by redrawing them fresh:
        # Trick: we can’t easily remove previous scatter artists individually; so replot them now:
        if self.points:
            xs, ys = zip(*self.points)
            self.ax.scatter(xs, ys, s=POINT_MARKER_SIZE)
        # labels are already maintained separately

def main():
    # ----- Basemap -----
    frame_gdf, ocean_gdf, land_gdf, coast_gdf, water_gdf = get_gdf_from_gpkg(
        GPKG_PATH, FRAME_LAYER, OCEAN_LAYER, LAND_LAYER, COAST_LAYER, WATER_LAYER
    )

    fig, ax = plt.subplots(figsize=(12, 8))

    if not land_gdf.empty:
        land_gdf.plot(ax=ax, facecolor="#1aac37", edgecolor="#000000", zorder=1)
    if not ocean_gdf.empty:
        ocean_gdf.plot(ax=ax, facecolor="#ceecfd", edgecolor="none", zorder=2)
    # if not water_gdf.empty:
    #     water_gdf.plot(ax=ax, facecolor="#a0c8f0", edgecolor="#74a8d8", linewidth=0.4, alpha=0.95, zorder=2)
    # if not coast_gdf.empty:
    #     coast_gdf.plot(ax=ax, color="#2f7f3f", linewidth=1.0, zorder=3)

    minx, miny, maxx, maxy = frame_gdf.total_bounds
    ax.set_xlim(minx, maxx); ax.set_ylim(miny, maxy)
    ax.set_aspect("equal"); ax.set_axis_off(); ax.margins(0)
    plt.subplots_adjust(left=0, right=1, top=1)

    # ----- Title / legend -----
    ax.set_title("Route Planner • Click to add waypoints • Buttons below to Finish/Undo/Clear/Cancel")
    try:
        ax.legend(loc='upper right', framealpha=0.8)
    except Exception:
        pass

    # ----- Route picker -----
    save_path = Path(get_ship_route_path(ROOT, ROUTE_FILENAME))
    rp = RoutePicker(ax, save_path=save_path)

    plt.show()

if __name__ == "__main__":
    main()

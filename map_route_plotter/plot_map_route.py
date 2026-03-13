from pathlib import Path
import sys
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

from matplotlib.lines import Line2D

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

mpl.rcParams["pdf.fonttype"] = 42
mpl.rcParams["ps.fonttype"] = 42
mpl.rcParams["font.family"] = "Arial"

from orchestrator.utils import get_ship_route_path_from_group, get_map_path
from map_route_plotter.prepare_map_route import get_gdf_from_gpkg, load_waypoints

LABEL_FONTSIZE = 7.5

def add_scalebar(ax, length_m=None, location=(0.08, 0.06), linewidth=3, text_offset=0.015):
    """
    Add a simple metric scale bar to an axes already in projected meters.
    """
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    width = x1 - x0
    height = y1 - y0

    if length_m is None:
        target = width * 0.15
        nice_lengths = np.array([500, 1000, 2000, 5000, 10000, 20000, 50000, 100000])
        length_m = nice_lengths[np.argmin(np.abs(nice_lengths - target))]

    sx = x0 + location[0] * width
    sy = y0 + location[1] * height

    ax.plot([sx, sx + length_m], [sy, sy], color="black", lw=linewidth, solid_capstyle="butt", zorder=20)
    ax.plot([sx, sx], [sy - 0.003 * height, sy + 0.003 * height], color="black", lw=linewidth, zorder=20)
    ax.plot([sx + length_m, sx + length_m], [sy - 0.003 * height, sy + 0.003 * height], color="black", lw=linewidth, zorder=20)

    label = f"{int(length_m/1000)} km" if length_m >= 1000 else f"{int(length_m)} m"
    ax.text(
        sx + length_m / 2,
        sy + text_offset * height,
        label,
        ha="center",
        va="bottom",
        fontsize=LABEL_FONTSIZE,
        bbox=dict(facecolor="white", edgecolor="none", alpha=0.8, pad=1.5),
        zorder=21,
    )


def plot_route_map(
    root: Path,
    route_filename: str,
    group: str=None,
    pattern: str= "*.txt",
    map_filename: str = "singapore_strait.gpkg",
    title: str = "Singapore Strait Route Map",
    fig_width: float = 12.0,
    dpi: int = 180,
    show_coast: bool = False,
    show_water: bool = False,
    show_waterways: bool = False,
    show_ferry_routes: bool = True,
    show_harbours: bool = False,
    show_bridges: bool = False,
    show_tss: bool = False,
    show_docks: bool = False,
    show: bool = True,
):
    """
    Plot route(s) on top of a prepared GeoPackage basemap.

    Parameters
    ----------
    root : Path
        Project root path.
    route_filename : str
        Route file stored under the route folder. If set to '*', means all route files
    group : str
        Map group associated to the route file(s)
    pattern : str
        Route file format. Default is .txt
    map_filename : str
        GeoPackage map filename stored under the map folder.
    title : str
        Figure title.
    fig_width : float
        Figure width in inches.
    dpi : int
        Figure DPI.
    show_coast, show_water, show_waterways, show_ferry_routes, show_harbours,
    show_bridges, show_tss, show_docks : bool
        Layer toggles.
    show : bool
        If True, call plt.show().

    Returns
    -------
    fig, ax
        Matplotlib figure and axes.
    """

    # -----------------------
    # Load route
    # -----------------------
    if route_filename == "*":
        route_files = get_ship_route_path_from_group(root=root, group=group, route_filename=route_filename, pattern=pattern)
        routes = [load_waypoints(file) for file in route_files]
    else:
        route_file = get_ship_route_path_from_group(root=root, group=group, route_filename=route_filename)
        routes = [load_waypoints(route_file)]

    # -----------------------
    # Load map layers
    # -----------------------
    gpkg_path = get_map_path(root, map_filename)

    frame_layer = "frame_3857"
    ocean_layer = "ocean_3857"
    land_layer = "land_3857"
    coast_layer = "coast_3857"
    water_layer = "water_3857"
    waterways_layer = "waterways_3857"
    ferry_routes_layer = "ferry_routes_3857"
    harbours_layer = "harbours_3857"
    bridges_layer = "bridges_3857"
    tss_layer = "tss_3857"
    docks_layer = "docks_3857"

    (
        frame_gdf, ocean_gdf, land_gdf, coast_gdf, water_gdf,
        waterways_gdf, ferry_routes_gdf, harbours_gdf, bridges_gdf, tss_gdf, docks_gdf
    ) = get_gdf_from_gpkg(
        gpkg_path,
        frame_layer=frame_layer,
        ocean_layer=ocean_layer,
        land_layer=land_layer,
        coast_layer=coast_layer,
        water_layer=water_layer,
        waterways_layer=waterways_layer,
        ferry_routes_layer=ferry_routes_layer,
        harbours_layer=harbours_layer,
        bridges_layer=bridges_layer,
        tss_layer=tss_layer,
        docks_layer=docks_layer,
    )

    # -----------------------
    # Figure size from map aspect
    # -----------------------
    minx, miny, maxx, maxy = frame_gdf.total_bounds
    map_w = maxx - minx
    map_h = maxy - miny
    aspect = map_w / map_h

    fig_height = fig_width / aspect
    fig, ax = plt.subplots(figsize=(fig_width, fig_height), dpi=dpi)

    # -----------------------
    # Basemap
    # -----------------------
    if not ocean_gdf.empty:
        ocean_gdf.plot(ax=ax, facecolor="#cfe8f7", edgecolor="none", zorder=0)

    if not land_gdf.empty:
        land_gdf.plot(ax=ax, facecolor="#dfe6d5", edgecolor="#7a8a6a", linewidth=0.30, zorder=1)

    if show_water and not water_gdf.empty:
        water_gdf.plot(ax=ax, facecolor="#b7dcef", edgecolor="none", zorder=2)

    if show_coast and not coast_gdf.empty:
        coast_gdf.plot(ax=ax, color="#4f6650", linewidth=0.45, zorder=3)

    # -----------------------
    # Optional overlays
    # -----------------------
    if show_waterways and not waterways_gdf.empty:
        waterways_gdf.plot(ax=ax, color="#7fb6d6", linewidth=0.6, alpha=0.9, zorder=4)

    if show_ferry_routes and not ferry_routes_gdf.empty:
        ferry_routes_gdf.plot(ax=ax, color="#5d6fd3", linewidth=0.25, linestyle="--", alpha=0.5, zorder=5)

    if show_tss and not tss_gdf.empty:
        tss_gdf.plot(ax=ax, color="#9c6ade", linewidth=1.0, linestyle=":", alpha=0.9, zorder=5)

    if show_bridges and not bridges_gdf.empty:
        bridges_gdf.plot(ax=ax, color="#6b4f3a", linewidth=1.2, alpha=0.9, zorder=6)

    if show_docks and not docks_gdf.empty:
        docks_gdf.plot(ax=ax, facecolor="#d9c27a", edgecolor="#8d7b45", linewidth=0.4, alpha=0.9, zorder=6)

    if show_harbours and not harbours_gdf.empty:
        harbours_gdf.plot(ax=ax, color="#c85a5a", markersize=14, alpha=0.85, zorder=7)

    # -----------------------
    # Routes
    # -----------------------
    palette = ["#0c3c78", "#d90808", "#2a9d8f", "#f4a261", "#6a4c93", "#264653"]
    route_handles = []
    for i, (north, east) in enumerate(routes):
        ax.plot(
            east, north,
            linestyle="--",
            linewidth=1.0,
            color=palette[i],
            zorder=10,
        )
        ax.scatter(
            east, north,
            s=20,
            color=palette[i],
            edgecolor="white",
            linewidth=0.5,
            zorder=11,
        )

        if (len(east) > 0) and (i==0):
            ax.text(east[0], north[0]+500, " START", fontsize=LABEL_FONTSIZE, weight="bold", va="bottom", ha="center", zorder=12)
            ax.text(east[-1], north[-1]+500, " GOAL", fontsize=LABEL_FONTSIZE, weight="bold", va="bottom", ha="center", zorder=12)

        route_handles.append(Line2D([0], [0], color=palette[i], lw=1.0, ls="--", label=f"Route {i+1}"))

    # -----------------------
    # Axes / grid / frame
    # -----------------------
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

    # -----------------------
    # Scalebar
    # -----------------------
    add_scalebar(ax, length_m=None)

    # -----------------------
    # Legend
    # -----------------------
    if len(route_handles) > 0:
        ax.legend(
            handles=route_handles,
            loc="upper left",
            frameon=True,
            facecolor="white",
            edgecolor="0.7",
            prop={'size':LABEL_FONTSIZE}
        )

    # -----------------------
    # Title and spacing
    # -----------------------
    ax.set_title(title, fontsize=LABEL_FONTSIZE, pad=10)

    plt.subplots_adjust(left=0.08, right=0.99, bottom=0.08, top=0.93)

    if show:
        plt.show()

    return fig, ax


def main():
    map = 1
    if map == 1:
        group           = "singapore-strait"
        map_filename    = "singapore_strait.gpkg"
        title           = "Singapore Strait Fleet Route Map"
    elif map == 2:
        group           = "oslo-fjord"
        map_filename    = "oslo_fjord.gpkg"
        title           = "Oslo Fjord Fleet Route Map"
    
    plot_route_map(
        root=ROOT,
        # route_filename="route.txt",
        route_filename="*",
        group=group,
        map_filename=map_filename,
        title=title,
        fig_width=7.0,
        dpi=180,
        show_coast=False,
        show_water=False,
        show_waterways=False,
        show_ferry_routes=True,
        show_harbours=False,
        show_bridges=False,
        show_tss=False,
        show_docks=False,
        show=True,
    )


if __name__ == "__main__":
    main()
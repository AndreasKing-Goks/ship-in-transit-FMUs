import os, sys
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation, FFMpegWriter
from matplotlib.ticker import FuncFormatter

import numpy as np

from orchestrator.utils import get_map_path
from map_route_plotter.prepare_map_route import get_gdf_from_gpkg, get_map_origin_lat_lon, get_polygon_from_gdf
from map_route_plotter.polygon_obstacle import PolygonObstacle

# =============================================================================================================
# Animation
# =============================================================================================================
class CopernicusWeatherAnimator:
    def __init__(
        self,
        ROOT : Path,
        map_filename,
        min_lon,
        min_lat,
        max_lon,
        max_lat,
    ):
        # Original latitude and longitude (Based on Norway's location)
        self.min_lon = min_lon
        self.min_lat = min_lat
        self.max_lon = max_lon
        self.max_lat = max_lat
        
        if self.is_map_exists:
            self.add_map(ROOT=ROOT, map_filename=map_filename)
            
            self.land_poly              = self.map_obj[0]
            self.frame_poly             = self.map_obj[1]

    
    def add_map(self, ROOT, map_filename):
        """
            Load map layers from a GeoPackage and store them as GeoDataFrames.
            Also converts the land layer into a Shapely polygon for later 
            geometric operations (e.g., grounding checks).
        """

        # Get full path to the GeoPackage file
        gpkg_path = get_map_path(ROOT, map_filename)

        # Layer names inside the GeoPackage (all projected in EPSG:3857)
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

        # Load layers from the GeoPackage into GeoDataFrames
        (
            self.frame_gdf, self.ocean_gdf, self.land_gdf, self.coast_gdf,
            self.water_gdf, self.waterways_gdf, self.ferry_routes_gdf,
            self.harbours_gdf, self.bridges_gdf, self.tss_gdf, self.docks_gdf
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
        
        # Update latitude and longitude based on the map
        self.lat0, self.lon0 = get_map_origin_lat_lon(gpkg_path=gpkg_path, frame_layer=frame_layer)
        
        # Merge land geometries into a single Shapely polygon
        # (useful for spatial checks such as grounding detection)
        land_poly       = get_polygon_from_gdf(self.land_gdf) 
        frame_poly      = get_polygon_from_gdf(self.frame_gdf) 
        self.map_obj    = [PolygonObstacle(land_poly), PolygonObstacle(frame_poly)]
    
    
    def set_map_static_aspect(self, frame_gdf, aspect: float=None):
        """
            Returns the static map plot aspect based on the frame_gdf.
            Also returns the map boundary. Else returns pre determined aspect.
        """
        if aspect is None:
            minx, miny, maxx, maxy = frame_gdf.total_bounds
            map_w = maxx - minx
            map_h = maxy - miny
            aspect = map_w / map_h
        return aspect, minx, miny, maxx, maxy
        
    def set_map_static_artist(self, ax):
        """
            Plot selected map layers on the provided Matplotlib axis.
            Visibility of each layer is controlled by the boolean flags.
        """
        ## Get the map details
        show_coast          = self.map.get("show_coast")
        show_water          = self.map.get("show_water")
        show_waterways      = self.map.get("show_waterways")
        show_ferry_routes   = self.map.get("show_ferry_routes")
        show_harbours       = self.map.get("show_harbours")
        show_bridges        = self.map.get("show_bridges")
        show_tss            = self.map.get("show_tss")
        show_docks          = self.map.get("show_docks")
        
        # Reuse the gdf for different plotting use
        frame_gdf           = self.frame_gdf
        ocean_gdf           = self.ocean_gdf
        land_gdf            = self.land_gdf 
        coast_gdf           = self.coast_gdf
        water_gdf           = self.water_gdf
        waterways_gdf       = self.waterways_gdf
        ferry_routes_gdf    = self.ferry_routes_gdf
        harbours_gdf        = self.harbours_gdf
        bridges_gdf         = self.bridges_gdf
        tss_gdf             = self.tss_gdf
        docks_gdf           = self.docks_gdf
        
        ## Base layers
        # Ocean background
        if not self.ocean_gdf.empty:
            ocean_gdf.plot(ax=ax, facecolor="#cfe8f7", edgecolor="none", zorder=0)

        # Land polygons
        if not self.land_gdf.empty:
            land_gdf.plot(ax=ax, facecolor="#dfe6d5", edgecolor="#7a8a6a", linewidth=0.30, zorder=1)

        # Optional water bodies
        if show_water and not self.water_gdf.empty:
            water_gdf.plot(ax=ax, facecolor="#b7dcef", edgecolor="none", zorder=2)

        # Coastline outline
        if show_coast and not self.coast_gdf.empty:
            coast_gdf.plot(ax=ax, color="#4f6650", linewidth=0.45, zorder=3)

        ## Optional overlays
        # Navigable waterways (rivers / channels)
        if show_waterways and not self.waterways_gdf.empty:
            waterways_gdf.plot(ax=ax, color="#7fb6d6", linewidth=0.6, alpha=0.9, zorder=4)

        # Ferry routes
        if show_ferry_routes and not self.ferry_routes_gdf.empty:
            ferry_routes_gdf.plot(ax=ax, color="#5d6fd3", linewidth=0.25, linestyle="--", alpha=0.5, zorder=5)

        # Traffic Separation Scheme
        if show_tss and not self.tss_gdf.empty:
            tss_gdf.plot(ax=ax, color="#9c6ade", linewidth=1.0, linestyle=":", alpha=0.9, zorder=5)

        # Bridges
        if show_bridges and not self.bridges_gdf.empty:
            bridges_gdf.plot(ax=ax, color="#6b4f3a", linewidth=1.2, alpha=0.9, zorder=6)

        # Dock areas
        if show_docks and not self.docks_gdf.empty:
            docks_gdf.plot(ax=ax, facecolor="#d9c27a", edgecolor="#8d7b45", linewidth=0.4, alpha=0.9, zorder=6)

        # Harbour markers
        if show_harbours and not self.harbours_gdf.empty:
            harbours_gdf.plot(ax=ax, color="#c85a5a", markersize=14, alpha=0.85, zorder=7)

        return (frame_gdf, ocean_gdf, land_gdf, coast_gdf,
                water_gdf, waterways_gdf, ferry_routes_gdf,
                harbours_gdf, bridges_gdf, tss_gdf, docks_gdf)

    def get_plot_style(self, mode="quick"):
        """
            Plotting conventions for both static plot and animation
        """
        if mode == "paper":
            return {
                "own_lw": 1.4,
                "route_lw": 1.1,
                "ship_lw": 1.5,

                "title_fs": 8,       # irrelevant if title disabled
                "label_fs": 7.5,
                "tick_fs": 6.5,
                "legend_fs": 5.8,

                "grid_alpha": 0.45,

                "dpi": 400,

                "startend_dy": 500.0,
            }

        elif mode == "quick":
            return {
                "own_lw": 1.2,
                "route_lw": 1.0,
                "ship_lw": 1.6,
                "title_fs": 10,
                "label_fs": 8,
                "tick_fs": 8,
                "legend_fs": 7,
                "grid_alpha": 0.8,
                "dpi": 110,
                "startend_dy": 500.0,
            }

        else:
            raise ValueError("mode must be 'quick' or 'paper'")
        

    def add_scalebar(self, ax, length_m=None, location=(0.08, 0.06), linewidth=3, text_offset=0.015, label_fs=11):
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
            fontsize=label_fs,
            bbox=dict(facecolor="white", edgecolor="none", alpha=0.8, pad=1.5),
            zorder=21,
        )
    
    
    def init_anim_figures(
        self,
        fig_width,
        dpi,
        equal_aspect,
        margin_frac,
        bounds=None,
        mode="quick",
    ):
        """
        Create animation figure that works both with map and without map.
        If map exists, use map bounds and resize figure height to map aspect.
        """
        style = self.get_plot_style(mode)

        ## If map exists
        if self.is_map_exists:
            # Temporary fig/ax just to obtain frame extent/aspect from your map helpers
            tmp_fig, tmp_ax = plt.subplots(figsize=(fig_width, fig_width), dpi=dpi)
            frame_gdf, _, _, _, _, _, _, _, _, _, _ = self.set_map_static_artist(ax=tmp_ax)
            aspect, minx, miny, maxx, maxy = self.set_map_static_aspect(frame_gdf)
            plt.close(tmp_fig)

            # Consider the status box height
            map_height = fig_width / aspect
            status_height = 0.10 * fig_width
            total_height = map_height + status_height

            fig = plt.figure(figsize=(fig_width, total_height), dpi=dpi, constrained_layout=True)
            gs = fig.add_gridspec(
                nrows=2,
                ncols=1,
                height_ratios=[map_height, status_height],
                hspace=0.0
            )
            ax_map = fig.add_subplot(gs[0, 0])
            ax_status = fig.add_subplot(gs[1, 0])
            ax_status.set_axis_off()

            # Draw map again on real axes
            self.set_map_static_artist(ax=ax_map)

            ax_map.set_xlim(minx, maxx)
            ax_map.set_ylim(miny, maxy)

        ## If no map included
        else:
            base_map_height = 0.90 * fig_width
            status_height = 0.10 * fig_width
            total_height = base_map_height + status_height

            fig = plt.figure(figsize=(fig_width, total_height), dpi=dpi, constrained_layout=True)
            gs = fig.add_gridspec(
                nrows=2,
                ncols=1,
                height_ratios=[base_map_height, status_height],
                hspace=0.0
            )
            ax_map = fig.add_subplot(gs[0, 0])
            ax_status = fig.add_subplot(gs[1, 0])
            ax_status.set_axis_off()

            if bounds is not None:
                x_min, x_max, y_min, y_max = bounds
                ax_map.set_xlim(x_min, x_max)
                ax_map.set_ylim(y_min, y_max)
        
        # Add scalebar on map
        self.add_scalebar(ax=ax_map, length_m=None, label_fs=style["label_fs"])

        ## Common styling
        title = f"Fleet trajectories on {self.map_name}" if self.is_map_exists else "Fleet trajectories"
        ax_map.set_title(title, fontsize=style["title_fs"], pad=4)
        ax_map.set_xlabel("East position (km)", fontsize=style["label_fs"])
        ax_map.set_ylabel("North position (km)", fontsize=style["label_fs"])

        ax_map.tick_params(axis="both", which="major", labelsize=style["tick_fs"], length=3)
        ax_map.grid(True, color="0.82", linestyle="--", linewidth=0.5, alpha=style["grid_alpha"])

        ax_map.ticklabel_format(style="sci", axis="both", scilimits=(0, 0))
        ax_map.xaxis.get_offset_text().set_fontsize(style["tick_fs"] - 1)
        ax_map.yaxis.get_offset_text().set_fontsize(style["tick_fs"] - 1)
        
        ax_map.xaxis.set_major_formatter(FuncFormatter(lambda value, pos: f'{value / 1000:g}'))
        ax_map.yaxis.set_major_formatter(FuncFormatter(lambda value, pos: f'{value / 1000:g}'))

        for spine in ax_map.spines.values():
            spine.set_linewidth(0.8)
            spine.set_color("0.35")

        if equal_aspect:
            ax_map.set_aspect("equal", adjustable="box")

        return fig, ax_map, ax_status
    
    
    ### HELPER FUNCTION FOR STATUS PANEL
    def safe_frame_value(self, seq, i, default=None):
        """
            Safely get value at frame i from a sequence-like object.
            If unavailable, return default.
        """
        if seq is None:
            return default
        if len(seq) == 0:
            return default
        if i < 0:
            return default
        if i >= len(seq):
            return seq[-1]
        return seq[i]
    
    def get_frame_value(self, seq, i):
        """
            Return value at frame i.
            If i exceeds length, return last available value.
            Assumes seq is non-empty.
        """
        return seq[i] if i < len(seq) else seq[-1]
    
    
    def get_stop_info_state_at_frame(self, ship_id, i):
        """
            Return a dictionary of stop/status information for a ship at frame i.
            Intended for OS0.
        """
        stop_info = getattr(self, "stop_info", {}).get(ship_id, None)
        if not stop_info:
            return None
        
        colav_status = self.get_frame_value(stop_info["colav_active"]["status"], i)
        colav_candidates = self.get_frame_value(stop_info["colav_active"]["candidates"], i)

        collision_status = self.get_frame_value(stop_info["collision"]["status"], i)
        collision_colliders = self.get_frame_value(stop_info["collision"]["colliders"], i)

        nav_warn = self.get_frame_value(stop_info["nav_fail_warning"], i)
        nav_fail = self.get_frame_value(stop_info["navigation_failure"], i)
        
        if self.skip_map_evaluation:
            grounding = False   # No grounding evaluation when skip map evaluation. Hence always output False
        else:
            grounding = self.get_frame_value(stop_info["grounding"], i)

        return {
            "colav_active": colav_status,
            "colav_candidates": colav_candidates,
            "collision": collision_status,
            "collision_colliders": collision_colliders,
            "grounding": grounding,
            "nav_warn": nav_warn,
            "nav_fail": nav_fail,
        }
        
        
    def format_status_detail(self, value, default="NONE"):
        """
            Convert stop-info detail field to a compact display string.
        """
        if value is None:
            return default

        if isinstance(value, str):
            v = value.strip()
            return v if v else default

        if isinstance(value, (list, tuple, set)):
            vals = [str(x).strip() for x in value if str(x).strip()]
            return ", ".join(vals) if vals else default

        v = str(value).strip()
        return v if v else default
    
    
    def init_status_panel_artists(self, ax_status, mode="quick"):
        """
            Create persistent status box artists for own ship:
            1. COLAV      (top + bottom)
            2. COLLISION  (top + bottom)
            3. NAVIGATION (single tall box)
            4. GROUNDING  (single tall box)
        """
        style = self.get_plot_style(mode)

        # Optional: tune these based on mode
        title_fs = max(7, style["label_fs"])
        status_fs = max(6, style["status_fs"]) * 0.8 
        detail_fs = max(6, style["tick_fs"])

        # Colors
        C_GREEN = "#7bd389"
        C_YELLOW = "#f4d35e"
        C_RED = "#ee6055"
        C_NEUTRAL = "#d9d9d9"
        C_EDGE = "0.35"

        # Layout in ax_status axes coordinates
        # Leave left margin for time/frame text
        x0 = 0.18
        gap = 0.02
        total_w = 0.80
        col_w = (total_w - 3 * gap) / 4.0

        y_bot = 0.12
        y_mid = 0.52
        h_small = 0.32
        h_tall = 0.72

        artists = {
            "time_patch": None,
            "time_text": None,
            "frame_patch": None,
            "frame_text": None,
            "colav_top_patch": None,
            "colav_top_text": None,
            "colav_bot_patch": None,
            "colav_bot_text": None,
            "collision_top_patch": None,
            "collision_top_text": None,
            "collision_bot_patch": None,
            "collision_bot_text": None,
            "nav_patch": None,
            "nav_text": None,
            "ground_patch": None,
            "ground_text": None,
            "dynamic_list": []
        }

        # ------------------------------------------------------------------
        # Time + frame stacked boxes at left
        # ------------------------------------------------------------------
        time_frame_x = 0.02
        time_frame_w = 0.14
        time_box_h = 0.32
        frame_box_h = 0.32

        # Time box
        time_patch = patches.Rectangle(
            (time_frame_x, y_mid), time_frame_w, time_box_h,
            transform=ax_status.transAxes,
            facecolor="white",
            edgecolor=C_EDGE,
            linewidth=1.0
        )
        ax_status.add_patch(time_patch)

        time_txt = ax_status.text(
            time_frame_x + time_frame_w / 2,
            y_mid + time_box_h / 2,
            "",
            transform=ax_status.transAxes,
            ha="center",
            va="center",
            fontsize=style["status_fs"]*0.8,
            weight="bold"
        )

        # Frame box
        frame_patch = patches.Rectangle(
            (time_frame_x, y_bot), time_frame_w, frame_box_h,
            transform=ax_status.transAxes,
            facecolor="white",
            edgecolor=C_EDGE,
            linewidth=1.0
        )
        ax_status.add_patch(frame_patch)

        frame_txt = ax_status.text(
            time_frame_x + time_frame_w / 2,
            y_bot + frame_box_h / 2,
            "",
            transform=ax_status.transAxes,
            ha="center",
            va="center",
            fontsize=style["status_fs"]*0.8,
            weight="bold"
        )

        artists["time_patch"] = time_patch
        artists["time_text"] = time_txt
        artists["frame_patch"] = frame_patch
        artists["frame_text"] = frame_txt
        artists["dynamic_list"].extend([time_patch, time_txt, frame_patch, frame_txt])

        # Column x positions
        x_colav = x0
        x_collision = x_colav + col_w + gap
        x_nav = x_collision + col_w + gap
        x_ground = x_nav + col_w + gap

        # ------------------------------------------------------------------
        # COLAV
        # ------------------------------------------------------------------
        colav_top = patches.Rectangle(
            (x_colav, y_mid), col_w, h_small,
            transform=ax_status.transAxes,
            facecolor=C_NEUTRAL, edgecolor=C_EDGE, linewidth=1.0
        )
        ax_status.add_patch(colav_top)

        colav_top_txt = ax_status.text(
            x_colav + col_w/2, y_mid + h_small/2,
            "OWN SHIP COLAV\nINACTIVE",
            transform=ax_status.transAxes,
            ha="center", va="center",
            fontsize=status_fs, weight="bold"
        )

        colav_bot = patches.Rectangle(
            (x_colav, y_bot), col_w, h_small,
            transform=ax_status.transAxes,
            facecolor="white", edgecolor=C_EDGE, linewidth=1.0
        )
        ax_status.add_patch(colav_bot)

        colav_bot_txt = ax_status.text(
            x_colav + col_w/2, y_bot + h_small/2,
            "NONE",
            transform=ax_status.transAxes,
            ha="center", va="center",
            fontsize=detail_fs
        )

        artists["colav_top_patch"] = colav_top
        artists["colav_top_text"] = colav_top_txt
        artists["colav_bot_patch"] = colav_bot
        artists["colav_bot_text"] = colav_bot_txt
        artists["dynamic_list"].extend([colav_top, colav_top_txt, colav_bot, colav_bot_txt])

        # ------------------------------------------------------------------
        # COLLISION
        # ------------------------------------------------------------------
        collision_top = patches.Rectangle(
            (x_collision, y_mid), col_w, h_small,
            transform=ax_status.transAxes,
            facecolor=C_NEUTRAL, edgecolor=C_EDGE, linewidth=1.0
        )
        ax_status.add_patch(collision_top)

        collision_top_txt = ax_status.text(
            x_collision + col_w/2, y_mid + h_small/2,
            "OWN SHIP COLLISION\nNO",
            transform=ax_status.transAxes,
            ha="center", va="center",
            fontsize=status_fs, weight="bold"
        )

        collision_bot = patches.Rectangle(
            (x_collision, y_bot), col_w, h_small,
            transform=ax_status.transAxes,
            facecolor="white", edgecolor=C_EDGE, linewidth=1.0
        )
        ax_status.add_patch(collision_bot)

        collision_bot_txt = ax_status.text(
            x_collision + col_w/2, y_bot + h_small/2,
            "NONE",
            transform=ax_status.transAxes,
            ha="center", va="center",
            fontsize=detail_fs
        )

        artists["collision_top_patch"] = collision_top
        artists["collision_top_text"] = collision_top_txt
        artists["collision_bot_patch"] = collision_bot
        artists["collision_bot_text"] = collision_bot_txt
        artists["dynamic_list"].extend([collision_top, collision_top_txt, collision_bot, collision_bot_txt])

        # ------------------------------------------------------------------
        # NAVIGATION (single tall box)
        # ------------------------------------------------------------------
        nav_patch = patches.Rectangle(
            (x_nav, y_bot), col_w, h_tall,
            transform=ax_status.transAxes,
            facecolor=C_NEUTRAL, edgecolor=C_EDGE, linewidth=1.0
        )
        ax_status.add_patch(nav_patch)

        nav_txt = ax_status.text(
            x_nav + col_w/2, y_bot + h_tall/2,
            "OWN SHIP NAVIGATION\nSAFE",
            transform=ax_status.transAxes,
            ha="center", va="center",
            fontsize=status_fs, weight="bold"
        )

        artists["nav_patch"] = nav_patch
        artists["nav_text"] = nav_txt
        artists["dynamic_list"].extend([nav_patch, nav_txt])

        # ------------------------------------------------------------------
        # GROUNDING (single tall box)
        # ------------------------------------------------------------------
        ground_patch = patches.Rectangle(
            (x_ground, y_bot), col_w, h_tall,
            transform=ax_status.transAxes,
            facecolor=C_NEUTRAL, edgecolor=C_EDGE, linewidth=1.0
        )
        ax_status.add_patch(ground_patch)

        ground_txt = ax_status.text(
            x_ground + col_w/2, y_bot + h_tall/2,
            "OWN SHIP GROUNDING\nNO",
            transform=ax_status.transAxes,
            ha="center", va="center",
            fontsize=status_fs, weight="bold"
        )

        artists["ground_patch"] = ground_patch
        artists["ground_text"] = ground_txt
        artists["dynamic_list"].extend([ground_patch, ground_txt])

        return artists
    
    
    def init_status_panel_disabled_artist(self, ax_status, mode="quick", status_panel_mode="AUTOMATICALLY_DISABLED", own_ship_id="OS0"):
        style = self.get_plot_style(mode)

        C_EDGE = "0.35"

        y_bot = 0.12
        y_mid = 0.52

        artists = {
            "time_patch": None,
            "time_text": None,
            "frame_patch": None,
            "frame_text": None,
            "disabled_patch": None,
            "disabled_text": None,
            "dynamic_list": []
        }

        # ------------------------------------------------------------------
        # Time + frame stacked boxes at left
        # ------------------------------------------------------------------
        time_frame_x = 0.02
        time_frame_w = 0.14
        time_box_h = 0.32
        frame_box_h = 0.32

        time_patch = patches.Rectangle(
            (time_frame_x, y_mid), time_frame_w, time_box_h,
            transform=ax_status.transAxes,
            facecolor="white",
            edgecolor=C_EDGE,
            linewidth=1.0
        )
        ax_status.add_patch(time_patch)

        time_txt = ax_status.text(
            time_frame_x + time_frame_w / 2,
            y_mid + time_box_h / 2,
            "",
            transform=ax_status.transAxes,
            ha="center",
            va="center",
            fontsize=style["status_fs"] * 0.8,
            weight="bold"
        )

        frame_patch = patches.Rectangle(
            (time_frame_x, y_bot), time_frame_w, frame_box_h,
            transform=ax_status.transAxes,
            facecolor="white",
            edgecolor=C_EDGE,
            linewidth=1.0
        )
        ax_status.add_patch(frame_patch)

        frame_txt = ax_status.text(
            time_frame_x + time_frame_w / 2,
            y_bot + frame_box_h / 2,
            "",
            transform=ax_status.transAxes,
            ha="center",
            va="center",
            fontsize=style["status_fs"] * 0.8,
            weight="bold"
        )

        artists["time_patch"] = time_patch
        artists["time_text"] = time_txt
        artists["frame_patch"] = frame_patch
        artists["frame_text"] = frame_txt
        artists["dynamic_list"].extend([time_patch, time_txt, frame_patch, frame_txt])

        # ------------------------------------------------------------------
        # Disabled message box
        # ------------------------------------------------------------------
        msg_x = 0.18
        msg_y = 0.12
        msg_w = 0.80
        msg_h = 0.72

        patch = patches.Rectangle(
            (msg_x, msg_y), msg_w, msg_h,
            transform=ax_status.transAxes,
            facecolor="white",
            edgecolor=C_EDGE,
            linewidth=1.0
        )
        ax_status.add_patch(patch)

        if status_panel_mode == "AUTOMATICALLY_DISABLED":
            message = (
                f"STATUS PANEL FOR {own_ship_id} IS AUTOMATICALLY DISABLED\n"
                "Set frame_step = 1 to enable status monitoring"
            )
        elif status_panel_mode == "DISABLED":
            message = "STATUS PANEL FOR {own_ship_id} IS DISABLED"
        else:
            message = "STATUS PANEL FOR {own_ship_id} IS DISABLED"

        txt = ax_status.text(
            msg_x + msg_w / 2,
            msg_y + msg_h / 2,
            message,
            transform=ax_status.transAxes,
            ha="center",
            va="center",
            fontsize=style["status_fs"],
            weight="bold"
        )

        artists["disabled_patch"] = patch
        artists["disabled_text"] = txt
        artists["dynamic_list"].extend([patch, txt])

        return artists
    
    
    def update_status_panel(self, i, artists, mode="quick", own_ship_id="OS0"):
        """
            Update the persistent status panel artists for own ship.
        """
        state = self.get_stop_info_state_at_frame(own_ship_id, i)

        # Colors
        C_GREEN = "#7bd389"
        C_YELLOW = "#f4d35e"
        C_RED = "#ee6055"
        C_NEUTRAL = "#d9d9d9"

        # Always update time text
        t_sec = i * self.stepSize
        artists["time_text"].set_text(f"TIME\n{int(round(t_sec))} s")
        artists["frame_text"].set_text(f"FRAME\n{i}")

        if state is None:
            # fallback/default display
            artists["colav_top_patch"].set_facecolor(C_NEUTRAL)
            artists["colav_top_text"].set_text("OWN SHIP COLAV\nUNKNOWN")
            artists["colav_bot_text"].set_text("NONE")

            artists["collision_top_patch"].set_facecolor(C_NEUTRAL)
            artists["collision_top_text"].set_text("OWN SHIP COLLISION\nUNKNOWN")
            artists["collision_bot_text"].set_text("NONE")

            artists["nav_patch"].set_facecolor(C_NEUTRAL)
            artists["nav_text"].set_text("OWN SHIP NAVIGATION\nUNKNOWN")

            artists["ground_patch"].set_facecolor(C_NEUTRAL)
            artists["ground_text"].set_text("OWN SHIP GROUNDING\nUNKNOWN")
            return

        # ------------------------------------------------------------------
        # COLAV
        # ------------------------------------------------------------------
        colav_active = bool(state["colav_active"])
        colav_candidates = self.format_status_detail(
            state["colav_candidates"],
            default="NONE"
        )

        if colav_active:
            artists["colav_top_patch"].set_facecolor(C_YELLOW)
            artists["colav_top_text"].set_text("OWN SHIP COLAV\nACTIVE")
            artists["colav_bot_text"].set_text(colav_candidates)
        else:
            artists["colav_top_patch"].set_facecolor(C_GREEN)
            artists["colav_top_text"].set_text("OWN SHIP COLAV\nINACTIVE")
            artists["colav_bot_text"].set_text("NONE")

        # ------------------------------------------------------------------
        # COLLISION
        # ------------------------------------------------------------------
        collision = bool(state["collision"])
        collision_colliders = self.format_status_detail(
            state["collision_colliders"],
            default="NONE"
        )

        if collision:
            artists["collision_top_patch"].set_facecolor(C_RED)
            artists["collision_top_text"].set_text("OWN SHIP COLLISION\nYES")
            artists["collision_bot_text"].set_text(collision_colliders)
        else:
            artists["collision_top_patch"].set_facecolor(C_GREEN)
            artists["collision_top_text"].set_text("OWN SHIP COLLISION\nNO")
            artists["collision_bot_text"].set_text("NONE")

        # ------------------------------------------------------------------
        # NAVIGATION
        # priority: failure > warning > safe
        # ------------------------------------------------------------------
        nav_fail = bool(state["nav_fail"])
        nav_warn = bool(state["nav_warn"])

        if nav_fail:
            artists["nav_patch"].set_facecolor(C_RED)
            artists["nav_text"].set_text("OWN SHIP NAVIGATION\nFAILURE")
        elif nav_warn:
            artists["nav_patch"].set_facecolor(C_YELLOW)
            artists["nav_text"].set_text("OWN SHIP NAVIGATION\nWARNING")
        else:
            artists["nav_patch"].set_facecolor(C_GREEN)
            artists["nav_text"].set_text("OWN SHIP NAVIGATION\nSAFE")

        # ------------------------------------------------------------------
        # GROUNDING
        # ------------------------------------------------------------------
        grounding = bool(state["grounding"])

        if grounding:
            artists["ground_patch"].set_facecolor(C_RED)
            artists["ground_text"].set_text("OWN SHIP GROUNDING\nYES")
        else:
            artists["ground_patch"].set_facecolor(C_GREEN)
            artists["ground_text"].set_text("OWN SHIP GROUNDING\nNO")
    
    
    def update_disabled_status_panel(self, i, artists):
        """
        Update time/frame boxes for disabled status panel.
        """
        t_sec = i * self.stepSize
        artists["time_text"].set_text(f"TIME\n{int(round(t_sec))} s")
        artists["frame_text"].set_text(f"FRAME\n{i}")
        
    
    ### HELPER FUNCTION FOR INTERMEDIATE WAPYOINT DATA HANDLING
    def points_to_xy(self, points):
        """
            Convert a list of points stored as [(north, east), ...]
            into plotting arrays x=east and y=north.
        """
        if not points:
            return np.array([]), np.array([])

        north = [p[0] for p in points]
        east  = [p[1] for p in points]
        return np.asarray(east, dtype=float), np.asarray(north, dtype=float)
    
    
    def points_to_offsets(self, points):
        """
            Convert a list of points stored as [(north, east), ...]
            into scatter offsets of shape (N, 2), where columns are [east, north].
        """
        if not points:
            return np.empty((0, 2), dtype=float)

        return np.asarray([[p[1], p[0]] for p in points], dtype=float)
    
    
    def iw_pairs_to_segment_xy(self, sampled_inter_wps, sampled_inter_wp_projs):
        """
            Convert paired lists of:
                sampled_inter_wps      = [(north, east), ...]
                sampled_inter_wp_projs = [(north, east), ...]
            into x/y arrays for a single Line2D artist using NaN-separated segments.
        """
        if not sampled_inter_wps or not sampled_inter_wp_projs:
            return np.array([]), np.array([])

        n_seg = min(len(sampled_inter_wps), len(sampled_inter_wp_projs))

        xs = []
        ys = []

        for k in range(n_seg):
            iw_n, iw_e = sampled_inter_wps[k]
            pj_n, pj_e = sampled_inter_wp_projs[k]

            xs.extend([iw_e, pj_e, np.nan])
            ys.extend([iw_n, pj_n, np.nan])

        return np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
    
    
    def get_iw_state_at_frame(self, ship_id, frame):
        """
            Return the latest IW animation snapshot for ship_id whose key <= frame.
            If none exists, return None.
        """
        ship_hist = getattr(self, "IW_sampling_data", {}).get(ship_id, None)
        if not ship_hist:
            return None

        valid_frames = [f for f in ship_hist.keys() if f <= frame]
        if not valid_frames:
            return None

        latest_frame = max(valid_frames)
        return ship_hist[latest_frame]
    
    
    def get_ship_roa(self, ship_id):
        """
        Get Radius of Acceptance (RoA) for a ship from ship_configs.
        Returns None if not available.
        """
        cfg = next((sc for sc in self.ship_configs if sc.get("id") == ship_id), None)
        if cfg is None:
            return None

        fmu_params = cfg.get("fmu_params", {})
        mm = fmu_params.get("MISSION_MANAGER", {})
        return mm.get("ra", None)
    
    
    def clear_inter_wp_roa_artists(self, sid, artists):
        """
        Remove previously drawn dynamic IW RoA circle patches for one ship.
        """
        if "inter_wp_roa" not in artists:
            return
        if sid not in artists["inter_wp_roa"]:
            return

        for circ in artists["inter_wp_roa"][sid]:
            try:
                circ.remove()
            except ValueError:
                # already removed or not attached
                pass

        artists["inter_wp_roa"][sid] = []
        
        
    def update_inter_wp_roa_artists(self, sid, artists, sampled_inter_wps, color, mode="quick"):
        """
        Rebuild dynamic RoA circles for sampled intermediate waypoints of one ship.
        Returns the newly created circle artists.
        """
        style = self.get_plot_style(mode)

        self.clear_inter_wp_roa_artists(sid, artists)

        ra = self.get_ship_roa(sid)
        if ra is None or ra <= 0:
            return []

        if not sampled_inter_wps:
            return []

        ax_map = artists["inter_wp"][sid].axes

        new_circles = []
        for iw_n, iw_e in sampled_inter_wps:
            circ = patches.Circle(
                (iw_e, iw_n),
                radius=ra,
                fill=True,
                color=color,
                alpha=style["roa_alpha"],
                zorder=2.5
            )
            ax_map.add_patch(circ)
            new_circles.append(circ)

        artists["inter_wp_roa"][sid] = new_circles
        return new_circles
    
    
    def update_iw_dynamic_artists(self, i, sid, artists, mode="quick", plot_inter_wp_roa=True, plot_inter_wp_proj=True):
        """
        Update IW-related artists for a given ship and frame.
        Returns any newly created artists that must also be redrawn.
        """
        extra_artists = []

        if not self.IW_sampling_animated:
            return extra_artists

        state = self.get_iw_state_at_frame(sid, i)

        if state is None:
            if sid in artists.get("active_path", {}):
                artists["active_path"][sid].set_data([], [])
            if sid in artists.get("inter_wp", {}):
                artists["inter_wp"][sid].set_offsets(np.empty((0, 2), dtype=float))
            if sid in artists.get("inter_wp_proj", {}):
                artists["inter_wp_proj"][sid].set_data([], [])
            if plot_inter_wp_roa:
                self.clear_inter_wp_roa_artists(sid, artists)
            return extra_artists

        active_path = state.get("active_path", [])
        sampled_inter_wps = state.get("sampled_inter_wps", [])
        sampled_inter_wp_projs = state.get("sampled_inter_wp_projs", [])

        x_path, y_path = self.points_to_xy(active_path)
        artists["active_path"][sid].set_data(x_path, y_path)

        offsets = self.points_to_offsets(sampled_inter_wps)
        artists["inter_wp"][sid].set_offsets(offsets)

        if plot_inter_wp_proj:
            x_proj, y_proj = self.iw_pairs_to_segment_xy(
                sampled_inter_wps,
                sampled_inter_wp_projs
            )
            artists["inter_wp_proj"][sid].set_data(x_proj, y_proj)
        else:
            artists["inter_wp_proj"][sid].set_data([], [])
        
        if plot_inter_wp_roa:
            color = artists["color"][sid]
            
            # Exclude the zeroeth IW from RoA drawing
            sampled_inter_wps_for_roa = sampled_inter_wps[1:] if len(sampled_inter_wps) > 1 else []

            
            new_circles = self.update_inter_wp_roa_artists(
                sid=sid,
                artists=artists,
                sampled_inter_wps=sampled_inter_wps_for_roa,
                color=color,
                mode=mode
            )
            extra_artists.extend(new_circles)

        return extra_artists
    
    
    ######
    
    
    def init_dynamic_artists(self, ax_map, ax_status, ship_ids, mode="quick", palette=None, with_labels=True, enable_status_panel=True, status_panel_mode="ENABLED"):
        """
        Dynamic artists: trajectory trail, ship outline, ship id label, status text.
        """
        style = self.get_plot_style(mode)

        if palette is None:
            palette = ["#0c3c78", "#d90808", "#2a9d8f", "#f4a261", "#6a4c93", "#264653"]

        if self.IW_sampling_animated:
            artists = {
                "trail": {},
                "outline": {},
                "label": {},
                "active_path": {},
                "inter_wp": {},
                "inter_wp_proj": {},
                "inter_wp_roa": {},
                "status_panel": None,
                "dynamic_list": [],
                "color": {}
            }
        else:
            artists = {
                "trail": {},
                "outline": {},
                "label": {},
                "status_panel": None,
                "dynamic_list": [],
                "color": {}
            }

        for k, sid in enumerate(ship_ids):
            color = palette[k % len(palette)]

            # Trajectory trail
            trail_line, = ax_map.plot([], [], lw=style["own_lw"], alpha=0.9, color=color)
            artists["trail"][sid] = trail_line
            artists["dynamic_list"].append(trail_line)

            # Ship outline
            dummy_xy = np.array([[0.0, 0.0],
                                [0.0, 0.0],
                                [0.0, 0.0]])
            poly = patches.Polygon(
                dummy_xy,
                closed=True,
                fill=False,
                lw=style["ship_lw"],
                ec=color,
                alpha=0.8
            )
            ax_map.add_patch(poly)
            artists["outline"][sid] = poly
            artists["dynamic_list"].append(poly)

            # Label
            if with_labels:
                txt = ax_map.text(
                    0.0, 0.0, sid,
                    fontsize=style["ship_label_fs"],
                    color=color,
                    zorder=15
                )
                artists["label"][sid] = txt
                artists["dynamic_list"].append(txt)
            
            if self.IW_sampling_animated:
                active_path_line, = ax_map.plot(
                    [], [],
                    lw=style["route_lw"],
                    ls="-.",
                    alpha=0.85,
                    color=color,
                    zorder=4
                )
                artists["active_path"][sid] = active_path_line
                artists["dynamic_list"].append(active_path_line)

                inter_wp = ax_map.scatter(
                    [], [],
                    s=style["waypoint_s"],
                    marker="o",
                    edgecolors="white",
                    color=color,
                    alpha=0.95,
                    zorder=5
                )
                artists["inter_wp"][sid] = inter_wp
                artists["dynamic_list"].append(inter_wp)

                inter_wp_proj_line, = ax_map.plot(
                    [], [],
                    lw=style["route_lw"],
                    ls=":",
                    alpha=0.9,
                    color=color,
                    zorder=4
                )
                artists["inter_wp_proj"][sid] = inter_wp_proj_line
                artists["dynamic_list"].append(inter_wp_proj_line)
                
                # Dynamic IW RoA circles
                artists["inter_wp_roa"][sid] = []
                
                # Colort
                artists["color"][sid] = color

        if enable_status_panel:
            status_panel = self.init_status_panel_artists(
                ax_status=ax_status,
                mode=mode
            )
        else:
            status_panel = self.init_status_panel_disabled_artist(
                ax_status=ax_status,
                mode=mode,
                status_panel_mode=status_panel_mode
            )

        artists["status_panel"] = status_panel
        artists["dynamic_list"].extend(status_panel["dynamic_list"])

        return artists
    
    
    def update_dynamic(
        self,
        i,
        ship_ids,
        artists,
        data,
        precomputed_outlines=None,
        trail_len=None,
        ship_scale=1.0,
        mode="quick",
        plot_inter_wp_roa=True,
        plot_inter_wp_proj=True,
        enable_status_panel=True
    ):
        returned_artists = list(artists["dynamic_list"])
        
        if artists.get("status_panel", None) is not None:
            if enable_status_panel:
                self.update_status_panel(
                    i=i,
                    artists=artists["status_panel"],
                    mode=mode,
                    own_ship_id="OS0"
                )
            else:
                self.update_disabled_status_panel(
                    i=i,
                    artists=artists["status_panel"]
                )

        for sid in ship_ids:
            east  = data[sid]["east"]
            north = data[sid]["north"]

            j0 = 0 if trail_len is None else max(0, i - int(trail_len))
            artists["trail"][sid].set_data(east[j0:i+1], north[j0:i+1])

            if precomputed_outlines is not None:
                artists["outline"][sid].set_xy(precomputed_outlines[sid][i])
            else:
                yaw = data[sid]["yaw"]
                draw = getattr(self, f"{sid}_draw")
                x_local, y_local = draw.local_coords(scale=ship_scale)
                x_rot, y_rot = draw.rotate_coords(x_local, y_local, yaw[i])
                x_tr, y_tr = draw.translate_coords(x_rot, y_rot, north[i], east[i])
                xy = np.column_stack([y_tr, x_tr])
                artists["outline"][sid].set_xy(xy)

            if sid in artists["label"]:
                artists["label"][sid].set_position((east[i], north[i]))

            if self.IW_sampling_animated:
                extra_artists = self.update_iw_dynamic_artists(
                    i=i,
                    sid=sid,
                    artists=artists,
                    mode=mode,
                    plot_inter_wp_roa=plot_inter_wp_roa,
                    plot_inter_wp_proj=plot_inter_wp_proj
                )
                returned_artists.extend(extra_artists)

        return returned_artists


    def AnimateFleetTrajectory(
        self,
        ship_ids=None,
        show=True,
        block=True,
        mode="quick",
        fig_width=7.0,
        margin_frac=0.08,
        equal_aspect=True,
        interval_ms=20,
        frame_step=1,
        show_status_panel=True,
        trail_len=300,
        plot_routes=True,
        exclude_target_ships_route= False,
        plot_waypoints=True,
        plot_roa=True,
        plot_start_end=True,
        plot_inter_wp_roa=True,
        plot_inter_wp_proj=True,
        with_labels=True,
        precompute_ship_outlines=True,
        save_path=None,
        writer_fps=20,
        palette=None,
        blit=True,
        ship_scale=1.0
    ):
        """
            Animate the fleet trajectories for one or more ships, with optional support
            for background map plotting, route overlays, waypoint markers, RoA circles,
            ship labels, and ship hull outlines.

            This method works in two modes:
            1. Without map:
            2. With map (`self.is_map_exists == True`):

            Parameters
            ----------
            ship_ids : list[str] or None, optional
                - If None, all ship IDs found in `self.ship_configs` are used.
            show : bool, optional
                Whether to display the animation window with `plt.show()`.
            block : bool, optional
                - True  -> the script waits until the plot window is closed
                - False -> the script continues immediately after showing the figure

            mode : {"quick", "paper"}, optional
                Selects plotting style presets.
                - `"quick"`:
                    lighter, faster, lower dpi, smaller fonts
                - `"paper"`:
                    larger fonts, thicker lines, higher dpi
                This affects:
                - line widths
                - font sizes
                - waypoint size
                - grid alpha
                - RoA alpha
                - dpi
            fig_width : float, optional
                Figure width in inches.
                Notes:
                - If a map exists, the figure height is adapted automatically to match
                the map aspect ratio, plus a small bottom status panel.
                - If no map exists, a default height proportional to `fig_width` is used.

            margin_frac : float, optional
                Margin fraction used only in the no-map case.
                When no map is present, trajectory bounds are computed from ship motion,
                then expanded by this fraction of the total span.
                Example:
                - `margin_frac=0.08` means add 8% padding around the data
                Ignored when map exists, because map bounds are used directly.

            equal_aspect : bool, optional
                Whether to enforce equal scaling on x and y axes.

            interval_ms : int or float, optional
                Delay between displayed animation frames in milliseconds.
                This controls interactive playback speed in the GUI.

            frame_step : int, optional
                Number of simulation frames to skip between animation frames.

            trail_len : int or None, optional
                Length of the visible trajectory trail behind each ship.
                - integer -> show only the last `trail_len` points
                - None    -> show the full trajectory from start up to current frame

            plot_routes : bool, optional
                Whether to draw each ship's planned route from `self.ship_configs`.
                
            plot_waypoints : bool, optional
                Whether to draw waypoint markers from the configured route.

            plot_roa : bool, optional
                Whether to draw Radius of Acceptance (RoA) circles around waypoints.

            plot_start_end : bool, optional
                Whether to annotate the first and last route points with text labels:
                `"START"` and `"END"`.

            with_labels : bool, optional
                Whether to draw ship ID text labels near the animated ship position.

            precompute_ship_outlines : bool, optional
                Whether to precompute the ship hull polygon for every frame before the
                animation starts.

            save_path : str or None, optional
                Output file path for saving the animation.

            writer_fps : int, optional
                Frames per second used when saving the animation to file.

            palette : list[str] or None, optional
                List of colors used cyclically for ships.
                Example:
                - `palette=["tab:blue", "tab:red", "tab:green"]`

            blit : bool, optional
                Whether to use Matplotlib blitting for faster animation redraw.
        """
        
        ship_ids = self.resolve_ship_ids(ship_ids)
        if len(ship_ids) == 0:
            raise ValueError("No valid ship ids to animate.")

        style = self.get_plot_style(mode)

        data, n_frames = self.prepare_playback_data(ship_ids)

        # Only needed for non-map case
        bounds = None

        if not self.is_map_exists:
            bounds = self.compute_square_bounds_no_map(
                data=data,
                ship_ids=ship_ids,
                margin_frac=margin_frac,
                include_routes=True,
                include_roa=True,
            )

        fig, ax_map, ax_status = self.init_anim_figures(
            fig_width=fig_width,
            dpi=style["dpi"],
            equal_aspect=equal_aspect,
            margin_frac=margin_frac,
            bounds=bounds,
            mode=mode
        )

        static_artists = self.draw_static(
            ax_map=ax_map,
            ship_ids=ship_ids,
            mode=mode,
            plot_routes=plot_routes,
            exclude_target_ships_route=exclude_target_ships_route,
            plot_waypoints=plot_waypoints,
            plot_roa=plot_roa,
            plot_start_end=plot_start_end,
            palette=palette
        )

        if plot_routes:
            leg = ax_map.legend(
                fontsize=style["legend_fs"],
                frameon=True,
                framealpha=0.75,
                borderpad=0.4,
                handlelength=2.2,
                loc="upper left"
            )
            leg.get_frame().set_linewidth(0.6)

        frame_step = max(1, int(frame_step))
        frames = range(0, n_frames, frame_step)
        
        if show_status_panel and frame_step == 1:
            status_panel_mode = "ENABLED"
        elif show_status_panel and frame_step != 1:
            status_panel_mode = "AUTOMATICALLY_DISABLED"
        else:
            status_panel_mode = "DISABLED"
            
        status_panel_enabled = status_panel_mode == "ENABLED"
        
        artists = self.init_dynamic_artists(
            ax_map=ax_map,
            ax_status=ax_status,
            ship_ids=ship_ids,
            mode=mode,
            palette=palette,
            with_labels=with_labels,
            enable_status_panel=status_panel_enabled,
            status_panel_mode=status_panel_mode
        )

        outlines = None
        if precompute_ship_outlines:
            outlines = self.precompute_outlines(data, ship_ids, n_frames, ship_scale)

        def update(i):
            return self.update_dynamic(
                i=i,
                ship_ids=ship_ids,
                artists=artists,
                data=data,
                precomputed_outlines=outlines,
                trail_len=trail_len,
                ship_scale=ship_scale,
                mode=mode,
                plot_inter_wp_roa=plot_inter_wp_roa,
                plot_inter_wp_proj=plot_inter_wp_proj,
                enable_status_panel=status_panel_enabled
            )

        self.ani = FuncAnimation(
            fig,
            update,
            frames=frames,
            interval=interval_ms,
            repeat=False,
            blit=blit
        )

        if save_path:
            writer = FFMpegWriter(fps=writer_fps)
            self.ani.save(save_path, writer=writer)

        if show:
            plt.show(block=block)

        return fig, ax_map, ax_status, self.ani
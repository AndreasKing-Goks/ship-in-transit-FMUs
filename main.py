# MANDATORY TO LOCATE THE .dll FILES
import os, sys
from pathlib import Path
from utils import CoSimInstance
from utils_draw import ShipDraw

dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

import matplotlib.pyplot as plt
import numpy as np

class ShipInTransitCoSimulation(CoSimInstance):
    '''
        This class is the extension of the CoSimInstance class
        especially built for running the FMU-based Ship In Transit Simulator
    '''
    def __init__(self,
                 autopilot_params : dict,
                 shaft_speed_controller_params  : dict,
                 throttle_controller_params     : dict,
                 machinery_system_params        : dict, 
                 rudder_params                  : dict,
                 ship_model_params              : dict,
                 set_points_manager_params      : dict,
                 start_north                    : float,
                 iw_north                       : list,
                 end_north                      : float,
                 start_east                     : float,
                 iw_east                        : list,
                 end_east                       : float,
                 instanceName                   : str     = "simulation",
                 stopTime                       : float   = 1.0, 
                 stepSize                       : float   = 0.01,
                 ship_plot_scale                : int     = 1):
        super().__init__(instanceName, stopTime, stepSize)
        
        # FMU params
        self.autopilot_params               = autopilot_params
        self.shaft_speed_controller_params  = shaft_speed_controller_params
        self.throttle_controller_params     = throttle_controller_params,
        self.machinery_system_params        = machinery_system_params, 
        self.rudder_params                  = rudder_params,
        self.ship_model_params              = ship_model_params,
        self.set_points_manager_params      = set_points_manager_params,
        
        # Ship Draw
        self.draw           = ShipDraw(ship_model_params["length_of_ship"],
                                       ship_model_params["width_of_ship"])
        self.x, self.y      = self.draw.local_coords(scale=ship_plot_scale)
        self.ship_drawings  = [[],[]]
        
        # Route
        self.route_north   = [start_north] + iw_north + [end_north]
        self.route_east    = [start_east]  + iw_east  + [end_east]
        
    
    def PlotShipTrajectory(
        self,
        show=True,
        mode="quick",   # "quick" or "paper"
        block=True,
        fig_width=5.0,
        every_n=5,
        margin_frac=0.08,     # 👈 8% padding around trajectory
        equal_aspect=True,
        save_path=None,
    ):
        # -------------------------
        # Plot Configurations
        # -------------------------
        if mode == "paper":
            own_lw    = 2.4
            route_lw  = 1.8
            ship_lw   = 2.0

            title_fs  = 12
            label_fs  = 11
            tick_fs   = 10
            legend_fs = 9

            grid_alpha = 0.4
            dpi = 500

        elif mode == "quick":
            own_lw    = 1.2
            route_lw  = 1.0
            ship_lw   = 1.6

            title_fs  = 9
            label_fs  = 8
            tick_fs   = 7
            legend_fs = 7

            grid_alpha = 0.2
            dpi = 110

        else:
            raise ValueError("mode must be 'quick' or 'paper'")
        
        # -------------------------
        # Fetch time series
        # -------------------------
        keys = ["north", "east", "yaw_angle_rad"]
        data = {}

        for key in keys:
            _, _, samples = self.GetObserverTimeSeries(key)
            data[key] = np.asarray(samples)

        north = data["north"]
        east  = data["east"]
        yaw   = data["yaw_angle_rad"]

        n = min(len(north), len(east), len(yaw))
        north, east, yaw = north[:n], east[:n], yaw[:n]

        # -------------------------
        # Indices for ship outlines
        # -------------------------
        every_n = max(1, int(every_n))
        idx = np.arange(0, n, every_n)

        # -------------------------
        # Prepare ship outlines
        # -------------------------
        for i in idx:
            x_ned, y_ned = self.draw.rotate_coords(self.x, self.y, yaw[i])
            x_tr,  y_tr  = self.draw.translate_coords(x_ned, y_ned, north[i], east[i])

            self.ship_drawings[0].append(x_tr)
            self.ship_drawings[1].append(y_tr)

        # -------------------------
        # Figure
        # -------------------------
        fig, ax = plt.subplots(figsize=(fig_width, fig_width), dpi=dpi)

        own_color   = "#0c3c78"
        route_color = "#d90808"

        # Trajectory
        ax.plot(east, north,
                lw=own_lw, color=own_color, label="Own ship")

        ax.plot(self.route_east, self.route_north,
                lw=route_lw, ls="--", color=route_color, label="Mission trajectory")

        ax.scatter(self.route_east, self.route_north,
                s=18 if mode == "quick" else 30,
                marker="x", color=route_color)

        # Ship drawings (subsample for speed)
        for x, y in zip(self.ship_drawings[1][::6],
                        self.ship_drawings[0][::6]):
            ax.plot(x, y, lw=ship_lw, color=own_color, alpha=0.6)

        # -------------------------
        # Zoom based on ROUTE (not actual ship path)
        # -------------------------
        rx_min, rx_max = min(self.route_east),  max(self.route_east)
        ry_min, ry_max = min(self.route_north), max(self.route_north)

        dx = rx_max - rx_min
        dy = ry_max - ry_min

        margin_frac = 0.08  # 8% padding

        ax.set_xlim(rx_min - margin_frac * dx, rx_max + margin_frac * dx)
        ax.set_ylim(ry_min - margin_frac * dy, ry_max + margin_frac * dy)

        # -------------------------
        # Styling (paper-friendly)
        # -------------------------
        ax.set_title("Ship trajectory", fontsize=title_fs, pad=4)
        ax.set_xlabel("East position (m)", fontsize=label_fs)
        ax.set_ylabel("North position (m)", fontsize=label_fs)

        ax.tick_params(axis="both", which="major", labelsize=tick_fs, length=3)
        ax.tick_params(axis="both", which="minor", labelsize=tick_fs-1, length=2)

        # Subtle grid
        ax.grid(True, which="major", linewidth=0.6, alpha=grid_alpha)
        ax.grid(True, which="minor", linewidth=0.4, alpha=grid_alpha * 0.5)

        # Legend: small, light, unobtrusive
        leg = ax.legend(
            fontsize=legend_fs,
            frameon=True,
            framealpha=0.75,
            borderpad=0.4,
            handlelength=2.2,
            loc="lower right"
        )
        leg.get_frame().set_linewidth(0.6)

        # Scientific notation (small)
        ax.ticklabel_format(style='sci', axis='both', scilimits=(0,0))
        ax.xaxis.get_offset_text().set_fontsize(tick_fs-1)
        ax.yaxis.get_offset_text().set_fontsize(tick_fs-1)

        if equal_aspect:
            ax.set_aspect("equal", adjustable="box")

        fig.tight_layout(pad=0.05)

        if save_path:
            fig.savefig(save_path, dpi=dpi, bbox_inches="tight")

        if show:
            plt.show(block=block)

        return fig, ax
        
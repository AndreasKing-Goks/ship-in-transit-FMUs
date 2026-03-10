# MANDATORY TO LOCATE THE .dll FILES
import os, sys
from pathlib import Path

dll_dir = Path(sys.prefix) / "Lib" / "site-packages" / "libcosimpy" / "libcosimc"
os.add_dll_directory(str(dll_dir))

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation, FFMpegWriter

import numpy as np

from orchestrator.cosim_instance import *
from orchestrator.utils import ShipDraw, compile_ship_params
import time

# =============================================================================================================
# =============================================================================================================
# Scheduler Instance
# =============================================================================================================
# =============================================================================================================

class ShipInTransitCoSimulation(CoSimInstance):
    '''
        This class is the extension of the CoSimInstance class
        especially built for running the FMU-based Ship In Transit Simulator
    '''
    def __init__(self,
                 config : dict,
                 ROOT   : Path
                 ):
        # =========================
        # Instantiate the Parent Class
        # =========================
        # Upack the config file
        simu_config     = config["simulation"]
        ship_configs    = config["ships"]
        
        # Name
        instanceName    = simu_config["instanceName"]

        # Time
        stopTime        = simu_config["stopTime"] # Number of steps in seconds (int)
        stepSize        = simu_config["stepSize"] # Number of seconds (int)
        
        # Initiate the parent class
        super().__init__(instanceName, stopTime, stepSize)
        
        # For colav_active printing flag
        self.print_col_msg = False
        
        # =========================
        # Build the Ships
        # =========================
        # Delayed status
        self.ship_delayed               = {ship_config["id"]: [] for ship_config in ship_configs}
        
        # Reach end waypoint status
        self.ship_reach_end_waypoint    = {ship_config["id"]: [] for ship_config in ship_configs}
        
        # # Colav active flag
        # self.ship_colav_active          = {ship_config["id"]: [] for ship_config in ship_configs}
        
        # Set up the FMUs for all ship assets
        self.AddAllShips(ship_configs=ship_configs, ROOT=ROOT)


# =============================================================================================================
# Add Ship Function
# =============================================================================================================
    def ship_slave(self, prefix: str, block: str) -> str:
        return f"{prefix}__{block}"
    
    
    def spawn_ship(self, ship_id:str, spawn: dict, fmu_params: dict):
        # Init the Ship Model with the spawn point
        fmu_params["SHIP_MODEL"]["initial_north_position_m"]      = spawn["north"]
        fmu_params["SHIP_MODEL"]["initial_east_position_m"]       = spawn["east"]
        fmu_params["SHIP_MODEL"]["initial_yaw_angle_rad"]         = np.deg2rad(spawn.get("yaw_angle_deg", 0.0))
        fmu_params["SHIP_MODEL"]["initial_forward_speed_m_per_s"] = spawn.get("forward_speed", 0.0)
        
        # Set the ship reach end waypoint status at False first
        self.ship_reach_end_waypoint[ship_id].append(False)
        
        # Start the ship when the time is equal to the start time, else ship is delayed
        start_time = spawn["start_time"]
        if self.time < start_time*1e9:
            self.ship_delayed[ship_id].append(True)
        else:
            self.ship_delayed[ship_id].append(False)
        
        return fmu_params
        
        
    def AddAllShips(self,
                    ship_configs: list,
                    ROOT: Path,):
        '''
            Add all configure ships and auto solve FMU connections 
            between each ship entity for the Collision Avoidance FMU
        '''
        
        # Store ship configs as the class attribute
        self.ship_configs = ship_configs
        
        for ship_config in ship_configs:
            spawn_in_route      = False if "spawn" in list(ship_config.keys()) else True
            prefix              = ship_config.get("id")
            SHIP_BLOCKS         = ship_config.get("SHIP_BLOCKS")
            SHIP_CONNECTIONS    = ship_config.get("SHIP_CONNECTIONS")
            SHIP_OBSERVERS      = ship_config.get("SHIP_OBSERVERS")
            fmu_params          = compile_ship_params(ship_config, spawn_in_route=spawn_in_route)
            enable_colav        = ship_config.get("enable_colav")
            
            # If not spawn in route
            if not spawn_in_route:
                spawn           = ship_config["spawn"]
                fmu_params = self.spawn_ship(ship_id=prefix, spawn=spawn, fmu_params=fmu_params)
            
            # Add slaves
            for block, relpath in SHIP_BLOCKS:
                self.AddSlave(
                    name=self.ship_slave(prefix, block),
                    path=str(ROOT / relpath)
                )

            # Set initial values
            for block, p in fmu_params.items():
                self.SetInitialValues(slaveName=self.ship_slave(prefix, block), params=p)

            # Add internal connections
            for in_block, in_var, out_block, out_var in SHIP_CONNECTIONS:
                self.AddSlaveConnection(
                    slaveInputName=self.ship_slave(prefix, in_block),
                    slaveInputVar=in_var,
                    slaveOutputName=self.ship_slave(prefix, out_block),
                    slaveOutputVar=out_var,
                )
            
            # Add COLAV FMU for the ship that has it
            if enable_colav:
                # All other ships (exclude self)
                targets = [sc for sc in ship_configs if sc.get("id") != prefix]

                for idx, tgt in enumerate(targets, start=1):
                    tgt_prefix = tgt.get("id")

                    self.AddSlaveConnection(
                        slaveInputName=self.ship_slave(prefix, "COLAV"),
                        slaveInputVar=f"tar_{idx}_north",
                        slaveOutputName=self.ship_slave(tgt_prefix, "SHIP_MODEL"),
                        slaveOutputVar="north",
                    )
                    self.AddSlaveConnection(
                        slaveInputName=self.ship_slave(prefix, "COLAV"),
                        slaveInputVar=f"tar_{idx}_east",
                        slaveOutputName=self.ship_slave(tgt_prefix, "SHIP_MODEL"),
                        slaveOutputVar="east",
                    )
                    self.AddSlaveConnection(
                        slaveInputName=self.ship_slave(prefix, "COLAV"),
                        slaveInputVar=f"tar_{idx}_yaw_angle",
                        slaveOutputName=self.ship_slave(tgt_prefix, "SHIP_MODEL"),
                        slaveOutputVar="yaw_angle_rad",
                    )
                    self.AddSlaveConnection(
                        slaveInputName=self.ship_slave(prefix, "COLAV"),
                        slaveInputVar=f"tar_{idx}_measured_speed",
                        slaveOutputName=self.ship_slave(tgt_prefix, "SHIP_MODEL"),
                        slaveOutputVar="total_ship_speed",
                    )
                    
            # Add observers (namespaced keys)
            for block, var, label in SHIP_OBSERVERS:
                self.AddObserverTimeSeriesWithLabel(
                    name=f"{prefix}.{var}",
                    slaveName=self.ship_slave(prefix, block),
                    variable=var,
                    var_label=label
                )

            # Add ShipDraw class
            draw_attr = f"{prefix}_draw"
            setattr(self, 
                    draw_attr, 
                    ShipDraw(fmu_params["SHIP_MODEL"].get("length_of_ship"), fmu_params["SHIP_MODEL"].get("width_of_ship")))
            
            
# =============================================================================================================
# Simulator Step Up
# =============================================================================================================
    def get_fmu_base_name(self, slave_name: str) -> str:
        """
        Example:
            TS1__MISSION_MANAGER -> MISSION_MANAGER
            OS2__SHIP_MODEL      -> SHIP_MODEL
        """
        if "__" in slave_name:
            return slave_name.split("__", 1)[1]
        return slave_name
    
    
    def get_masked_input_val_for_delayed_ship(self, ship_id, in_slave, in_var):
        """
        Return the value to be written into an delayed ship's input port.
        This masks the ship's incoming signals so subsystems do not evolve
        in a meaningful way during the delayed phase.
        """
        # Get the base FMU name
        in_slave_base_name = self.get_fmu_base_name(in_slave)
        
        ship_model_name = self.ship_slave(ship_id, "SHIP_MODEL")

        # Inputs that should track the ship's own frozen state
        passthrough_from_ship_model = {
            ("MISSION_MANAGER", "north"): "north",
            ("MISSION_MANAGER", "east"): "east",

            ("COLAV", "own_north"): "north",
            ("COLAV", "own_east"): "east",
            ("COLAV", "own_yaw_angle"): "yaw_angle_rad",
            ("COLAV", "own_measured_speed"): "total_ship_speed",
        }

        key = (in_slave_base_name, in_var)
        if key in passthrough_from_ship_model:
            return self.GetLastValue(
                slaveName=ship_model_name,
                slaveVar=passthrough_from_ship_model[key]
            )

        # Inputs that should simply be zeroed
        zero_mask = {
            ("AUTOPILOT", "north"),
            ("AUTOPILOT", "east"),
            ("AUTOPILOT", "yaw_angle_rad"),
            ("AUTOPILOT", "prev_wp_north"),
            ("AUTOPILOT", "prev_wp_east"),
            ("AUTOPILOT", "next_wp_north"),
            ("AUTOPILOT", "next_wp_east"),

            ("SHAFT_SPEED_CONTROLLER", "desired_ship_speed"),
            ("SHAFT_SPEED_CONTROLLER", "measured_ship_speed"),

            ("THROTTLE_CONTROLLER", "desired_shaft_speed_rpm"),
            ("THROTTLE_CONTROLLER", "measured_shaft_speed_rpm"),

            ("SPEED_CONTROLLER", "desired_ship_speed"),
            ("SPEED_CONTROLLER", "measured_ship_speed"),

            ("MACHINERY_SYSTEM", "load_perc"),

            ("RUDDER", "rudder_angle_deg"),
            ("RUDDER", "yaw_angle_rad"),
            ("RUDDER", "forward_speed"),
            ("RUDDER", "current_speed"),
            ("RUDDER", "current_dir_rad"),

            ("SHIP_MODEL", "thrust_force"),
            ("SHIP_MODEL", "rudder_force_v"),
            ("SHIP_MODEL", "rudder_force_r"),
            ("SHIP_MODEL", "wind_speed"),
            ("SHIP_MODEL", "wind_dir_rad"),
            ("SHIP_MODEL", "current_speed"),
            ("SHIP_MODEL", "current_dir_rad"),

            ("COLAV", "throttle_cmd"),
            ("COLAV", "rudder_angle_deg"),
        }

        if key in zero_mask:
            return 0.0

        # Pattern-based masking for COLAV targets
        # Set to a far away location for all target ship to prevent triggering the collision avoidance
        if in_slave == "COLAV":
            if in_var.startswith("tar_") and in_var.endswith("_north"):
                return 1e9
            if in_var.startswith("tar_") and in_var.endswith("_east"):
                return 1e9
            if in_var.startswith("tar_") and in_var.endswith("_yaw_angle"):
                return 0.0
            if in_var.startswith("tar_") and in_var.endswith("_measured_speed"):
                return 0.0

        raise KeyError(
            f"No inactive-mask rule defined for ship_id={ship_id}, "
            f"in_slave={in_slave}, in_var={in_var}"
        )
    
    
    def CoSimManipulate(self):
        for i in range(len(self.slave_input_name)):
            try:
                in_slave = self.slave_input_name[i]
                in_var   = self.slave_input_var[i]
                out_slave= self.slave_output_name[i]
                out_var  = self.slave_output_var[i]

                ship_id = in_slave[:3]
                is_not_delayed = not self.ship_delayed.get(ship_id)[-1]

                out_vr, out_type = GetVariableInfo(self.slaves_variables[out_slave], out_var)
                in_vr,  in_type  = GetVariableInfo(self.slaves_variables[in_slave],  in_var)

                if out_type != in_type:
                    continue  # or raise
                
                if is_not_delayed:
                    out_val = [self.GetLastValue(slaveName=out_slave, slaveVar=out_var)]
                else:
                    out_val = [self.get_masked_input_val_for_delayed_ship(ship_id=ship_id,
                                                                          in_slave=in_slave,
                                                                          in_var=in_var)]

                if out_type == CosimVariableType.REAL:
                    self.manipulator.slave_real_values(self.slaves_index[in_slave], [in_vr], out_val)
                elif out_type == CosimVariableType.BOOLEAN:
                    self.manipulator.slave_boolean_values(self.slaves_index[in_slave], [in_vr], out_val)
                elif out_type == CosimVariableType.INTEGER:
                    self.manipulator.slave_integer_values(self.slaves_index[in_slave], [in_vr], out_val)
                else:
                    self.manipulator.slave_string_values(self.slaves_index[in_slave], [in_vr], out_val)

            except Exception as error:
                print("An error occured during signal manipulation: ",
                    self.slave_input_name[i], ".", self.slave_input_var[i], " = ",
                    self.slave_output_name[i], ".", self.slave_output_var[i], " :",
                    type(error).__name__, "-", error)
                sys.exit(1)
    
    
    def PreSolverFunctionCall(self):
        """
        Function to handle:
        - External input handling (such as RL-action)
        - Additional FMU manipulation using given external input
        """
        pass
    
    
    def update_ship_reach_end_point_status(self, ship_id):
        # Get the reach end waypoint flag
        rew_flag = self.GetLastValue(slaveName=self.ship_slave(ship_id, "MISSION_MANAGER"), 
                                     slaveVar="reach_wp_end")
        
        # If reach end waypoint, set the flag as True
        if rew_flag:
            self.ship_reach_end_waypoint[ship_id].append(True)
        
        return rew_flag
    
    
    def update_ship_delayed_status(self, ship_id, spawn):
        start_time  = spawn.get("start_time")
        
        if self.time < start_time*1e9:
            self.ship_delayed[ship_id].append(True)
        else:
            self.ship_delayed[ship_id].append(False)
            

    def update_colav_active_flag(self, ship_id):
        return self.GetLastValue(slaveName=self.ship_slave(ship_id, "COLAV"), 
                                 slaveVar="colav_active")
        
    
    def update_collision_status(self, ship_id):
        return self.GetLastValue(slaveName=self.ship_slave(ship_id, "COLAV"), 
                                 slaveVar="ship_collision")
        

    def PostSolverFunctionCall(self):
        """
        Function to handle:
        - Simulation termination assesment and handling
        - Reward Function for reward signal (RL only)
        """
        # Get the reach end point and collision flag for all assets
        rew_flags       = {ship_config["id"]: False for ship_config in self.ship_configs}
        collision_flags = {ship_config["id"]: False for ship_config in self.ship_configs}
        
        for ship_config in self.ship_configs:
            ship_id = ship_config.get("id")
            spawn   = ship_config["spawn"] if "spawn" in list(ship_config.keys()) else None
            
            # Update ship active status
            if spawn is not None:
                self.update_ship_delayed_status(ship_id, spawn)
            
            # Update reach end point status
            rew_flag  = self.update_ship_reach_end_point_status(ship_id)
            rew_flags[ship_id] = rew_flag
            
            if ship_config.get("enable_colav"):
                # Update colav active flag
                colav_active    = self.update_colav_active_flag(ship_id)
                
                # Update collision status
                collision_flag  = self.update_collision_status(ship_id)
                collision_flags[ship_id] = collision_flag
                
                # Update message
                if colav_active and (not collision_flag) and (not self.print_col_msg):
                    print(f"{ship_id}_COLAV is active!")
                    self.print_col_msg = True
                elif (not colav_active) and self.print_col_msg:
                    print(f"{ship_id}_COLAV is deactivated!")
                    self.print_col_msg = False
                elif collision_flag:
                    print(f"{ship_id} collides!")
        
        all_ship_reaches_end_point = np.all([rew_flags[ship_id] for ship_id in list(rew_flags.keys())])
        any_ship_collides          = np.any([collision_flags[ship_id] for ship_id in list(collision_flags.keys())])
        
        # Conclude the stop flag
        self.stop = all_ship_reaches_end_point or any_ship_collides
        

    def step(self):
        # Simulate
        self.CoSimManipulate()
        self.SetInputFromExternal()
        self.PreSolverFunctionCall()
        self.execution.step()
        self.PostSolverFunctionCall()
    
    
    def Simulate(self):
        """
        Output simulation time
        """
        # Start timer
        start_time = time.perf_counter()
        
        while self.time <  self.stopTime: 
            self.step()
            
             # Integrate or stop the simulator
            if not self.stop:
                self.time +=self.stepSize
            else:
                break
        
        # Stop timer
        end_time = time.perf_counter()
        
        # Compute elapsed time
        elapsed_time = end_time -start_time
        
        print(f"Simulation took {elapsed_time:.6f} seconds.")


# =============================================================================================================
# Static Plot
# =============================================================================================================
    def _get_ship_timeseries(self, ship_id: str, var: str):
        key = f"{ship_id}.{var}"
        t, step, samples = self.GetObserverTimeSeries(key)
        return np.asarray(t), np.asarray(step), np.asarray(samples)

    
    def PlotFleetTrajectory(
        self,
        ship_ids=None,          # None = plot all ships in self.ship_configs
        show=True,
        mode="quick",
        block=True,
        fig_width=10.0,
        every_n=25,
        margin_frac=0.08,
        equal_aspect=True,
        save_path=None,
        plot_routes=True,
        plot_waypoints=True,
        plot_outlines=True,
    ):
        # Pick ships
        if ship_ids is None:
            ship_ids = [sc.get("id") for sc in self.ship_configs]
        ship_ids = [sid for sid in ship_ids if sid is not None]

        # Plot configurations (reuse yours)
        if mode == "paper":
            own_lw, route_lw, ship_lw = 2.4, 1.8, 2.0
            title_fs, label_fs, tick_fs, legend_fs = 12, 11, 10, 9
            grid_alpha, roa_alpha, dpi = 0.4, 0.25, 500
        elif mode == "quick":
            own_lw, route_lw, ship_lw = 1.2, 1.0, 1.6
            title_fs, label_fs, tick_fs, legend_fs = 9, 8, 7, 7
            grid_alpha, roa_alpha, dpi = 0.2, 0.15, 110
        else:
            raise ValueError("mode must be 'quick' or 'paper'")

        every_n = max(1, int(every_n))

        # Figure
        fig, ax = plt.subplots(figsize=(fig_width, fig_width), dpi=dpi)

        # simple palette (you can replace with your own, incl. colorblind-safe)
        palette = ["#0c3c78", "#d90808", "#2a9d8f", "#f4a261", "#6a4c93", "#264653"]

        # track global extents (route or traj)
        all_x, all_y = [], []

        # per-ship plotting
        for k, sid in enumerate(ship_ids):
            color = palette[k % len(palette)]

            # --- time series ---
            _, _, north = self._get_ship_timeseries(sid, "north")
            _, _, east  = self._get_ship_timeseries(sid, "east")
            _, _, yaw   = self._get_ship_timeseries(sid, "yaw_angle_rad")
            
            n = min(len(north), len(east), len(yaw))
            north, east, yaw = north[:n], east[:n], yaw[:n]

            all_x.append(east)
            all_y.append(north)
            
            # --- trajectory ---
            ax.plot(east, north, lw=own_lw, color=color, label=f"{sid} trajectory")

            # --- route per ship (from config) ---
            if plot_routes:
                print("Drawing routes ...")
                cfg = next((sc for sc in self.ship_configs if sc.get("id") == sid), None)
                if cfg and "route" in cfg:
                    rN = cfg["route"].get("north", [])
                    rE = cfg["route"].get("east", [])
                    if len(rN) and len(rE):
                        ax.plot(rE, rN, lw=route_lw, ls="--", color=color, alpha=0.7,
                                label=f"{sid} route")
                        if plot_waypoints:
                            ax.scatter(rE, rN,
                                    s=18 if mode == "quick" else 30,
                                    marker="x", color=color)

                            # RoA circles
                            ra = cfg["fmu_params"].get("MISSION_MANAGER", {}).get("ra", None)
                            if ra is not None:
                                print("Drawing RoA ...")
                                for n_wp, e_wp in zip(rN[1:], rE[1:]):
                                    circ = patches.Circle(
                                        (e_wp, n_wp),
                                        radius=ra,
                                        fill=True,
                                        color=color,
                                        alpha=roa_alpha
                                    )
                                    ax.add_patch(circ)

            # --- ship outlines ---
            if plot_outlines:
                print("Drawing ship outlines ...")
                idx = np.arange(0, n, every_n)
                draw_attr = f"{sid}_draw"
                draw = getattr(self, draw_attr)
                for i in idx[::2]:  # extra subsample for speed
                    x, y = draw.local_coords()
                    x_ned, y_ned = draw.rotate_coords(x, y, yaw[i])
                    x_tr,  y_tr  = draw.translate_coords(x_ned, y_ned, north[i], east[i])
                    ax.plot(y_tr, x_tr, lw=ship_lw, color=color, alpha=0.6)

        # Zoom (global)
        X = np.concatenate(all_x) if len(all_x) else np.array([0.0, 1.0])
        Y = np.concatenate(all_y) if len(all_y) else np.array([0.0, 1.0])

        x_min, x_max = float(np.min(X)), float(np.max(X))
        y_min, y_max = float(np.min(Y)), float(np.max(Y))

        dx = max(1e-9, x_max - x_min)
        dy = max(1e-9, y_max - y_min)

        ax.set_xlim(x_min - margin_frac * dx, x_max + margin_frac * dx)
        ax.set_ylim(y_min - margin_frac * dy, y_max + margin_frac * dy)

        # Styling
        ax.set_title("Fleet trajectories", fontsize=title_fs, pad=4)
        ax.set_xlabel("East position (m)", fontsize=label_fs)
        ax.set_ylabel("North position (m)", fontsize=label_fs)

        ax.tick_params(axis="both", which="major", labelsize=tick_fs, length=3)
        ax.grid(True, which="major", linewidth=0.6, alpha=grid_alpha)

        leg = ax.legend(fontsize=legend_fs, frameon=True, framealpha=0.75,
                        borderpad=0.4, handlelength=2.2, loc="lower center")
        leg.get_frame().set_linewidth(0.6)

        ax.ticklabel_format(style='sci', axis='both', scilimits=(0,0))
        ax.xaxis.get_offset_text().set_fontsize(tick_fs-1)
        ax.yaxis.get_offset_text().set_fontsize(tick_fs-1)

        if equal_aspect:
            ax.set_aspect("equal", adjustable="box")

        fig.tight_layout(pad=0.05)

        if save_path:
            fig.savefig(save_path, dpi=dpi, bbox_inches="tight")

        if show:
            print("Plot is finished!")
            plt.tight_layout()
            plt.show(block=block)

        return fig, ax
    

# =============================================================================================================
# Animation
# =============================================================================================================
    def _resolve_ship_ids(self, ship_ids):
        if ship_ids is None:
            ship_ids = [sc.get("id") for sc in self.ship_configs]
        # filter None + keep order + remove duplicates
        out = []
        seen = set()
        for sid in ship_ids:
            if sid is None:
                continue
            if sid in seen:
                continue
            seen.add(sid)
            out.append(sid)
        return out
    
    
    def _compute_bounds_from_playback_data(self, data, ship_ids):
        all_e = []
        all_n = []
        for sid in ship_ids:
            all_e.append(np.asarray(data[sid]["east"]))
            all_n.append(np.asarray(data[sid]["north"]))

        E = np.concatenate(all_e) if all_e else np.array([0.0, 1.0])
        N = np.concatenate(all_n) if all_n else np.array([0.0, 1.0])

        x_min, x_max = float(np.min(E)), float(np.max(E))
        y_min, y_max = float(np.min(N)), float(np.max(N))
        return (x_min, x_max, y_min, y_max)
    
    
    def _precompute_outlines(self, data, ship_ids, n_frames):
        """
        Returns:
            outlines[sid] = list of xy arrays, length n_frames
            where xy is shape (n_pts, 2) in [east, north] columns.
        """
        outlines = {}

        for sid in ship_ids:
            draw = getattr(self, f"{sid}_draw")
            east  = np.asarray(data[sid]["east"])
            north = np.asarray(data[sid]["north"])
            yaw   = np.asarray(data[sid]["yaw"])

            # local hull coords (constant per ship)
            x_local, y_local = draw.local_coords()

            xy_frames = []
            for i in range(n_frames):
                x_rot, y_rot = draw.rotate_coords(x_local, y_local, yaw[i])
                x_tr,  y_tr  = draw.translate_coords(x_rot, y_rot, north[i], east[i])

                # IMPORTANT:
                # In your earlier plotting you used ax.plot(y_tr, x_tr),
                # meaning y_tr behaves like East and x_tr behaves like North.
                # Therefore: xy = [East, North] = [y_tr, x_tr]
                xy = np.column_stack([y_tr, x_tr])
                xy_frames.append(xy)

            outlines[sid] = xy_frames

        return outlines
    
    
    def _prepare_playback_data(self, ship_ids):
        data = {}
        
        for sid in ship_ids:
            # Unload data
            _, _, north   = self._get_ship_timeseries(sid, "north")
            _, _, east    = self._get_ship_timeseries(sid, "east")
            _, _, yaw     = self._get_ship_timeseries(sid, "yaw_angle_rad")
            
            # Align data lengths
            n = min(len(north), len(east), len(yaw))
            north, east, yaw = north[1:n], east[1:n], yaw[1:n]
            
            # Prepare per ship datum
            datum = {
                "north" : north, 
                "east"  : east, 
                "yaw"   : yaw, 
                "N"     : n-1
                }
            
            # Add datum to data container
            data[sid] = datum
        
        # Get the number of simulation frames.
        n_frames = min(data[sid]["N"] for sid in ship_ids)
        
        return data, n_frames
    

    def _init_anim_figures(self, fig_width, equal_aspect, margin_frac, bounds=None):
        # Prepare gid an axes
        fig     = plt.figure(figsize=(fig_width, 0.9 * fig_width), constrained_layout=True)
        gs      = fig.add_gridspec(nrows=2, ncols=1, height_ratios=[1.0, 0.12], hspace=0.0)
        ax_map  = fig.add_subplot(gs[0, 0])
        ax_status = fig.add_subplot(gs[1, 0]); ax_status.set_axis_off()
        
        ## Ax map styles
        ax_map.set_title("Fleet Trajectory Animation")
        ax_map.grid(alpha=0.25)
        ax_map.set_xlabel("East (m)")
        ax_map.set_ylabel("North (m)")
        
        # Unpack bounds
        if bounds is not None:
            x_min, x_max, y_min, y_max = bounds
            
            dx = x_max -x_min
            dy = y_max - y_min
            
            # Robust minimum span
            min_span = 1.0
            if dx < min_span:
                cx = 0.5 * (x_min + x_max)
                x_min, x_max = cx - 0.5 * min_span, cx + 0.5 * min_span
                dx = x_max - x_min
            if dy < min_span:
                cy = 0.5 * (y_min + y_max)
                y_min, y_max = cy - 0.5 * min_span, cy + 0.5 * min_span
                dy = y_max - y_min
            
            ax_map.set_xlim(x_min - margin_frac*dx, x_max + margin_frac*dx)
            ax_map.set_ylim(y_min - margin_frac*dy, y_max + margin_frac*dy)
            
        # Set equal aspect if told so
        if equal_aspect:
            ax_map.set_aspect("equal", adjustable="box")
            
        return fig, ax_map, ax_status
    

    def _draw_static(self, ax_map, ship_ids, plot_routes, plot_waypoints, plot_roa, palette=None):
        if (not plot_routes) and (not plot_waypoints) and (not plot_roa):
            return []

        if palette is None:
            palette = ["#0c3c78", "#d90808", "#2a9d8f", "#f4a261", "#6a4c93", "#264653"]

        static_artists = []

        for k, sid in enumerate(ship_ids):
            color = palette[k % len(palette)]

            cfg = next((sc for sc in self.ship_configs if sc.get("id") == sid), None)
            if not cfg:
                continue

            route = cfg.get("route", None)
            if not route:
                continue

            rN = route.get("north", [])
            rE = route.get("east",  [])
            m = min(len(rN), len(rE))
            if m == 0:
                continue
            rN = rN[:m]
            rE = rE[:m]

            if plot_routes:
                line, = ax_map.plot(rE, rN, alpha=0.7, color=color, ls="--", label=f"{sid} route", zorder=2)
                static_artists.append(line)

            if plot_waypoints:
                wp = ax_map.scatter(rE, rN, marker="x", color=color, zorder=3)
                static_artists.append(wp)

            if plot_roa:
                fmu_params = cfg.get("fmu_params", {})  # <-- underscore
                mm = fmu_params.get("MISSION_MANAGER", {})
                ra = mm.get("ra", None)

                if ra is not None and ra > 0:
                    for n_wp, e_wp in zip(rN[1:], rE[1:]):
                        circ = patches.Circle((e_wp, n_wp), radius=ra, fill=True,
                                            alpha=0.25, color=color, zorder=1)
                        ax_map.add_patch(circ)
                        static_artists.append(circ)

        return static_artists

    
    def _init_dynamic_artists(self, ax_map, ax_status, ship_ids, palette=None, with_labels=True):
        # palette is optional; just keep consistent colors per ship
        if palette is None:
            palette = ["#0c3c78", "#d90808", "#2a9d8f", "#f4a261", "#6a4c93", "#264653"]

        # Artists container
        artists = {
            "trail": {},            # sid -> Line2D
            "outline": {},          # sid -> Polygon (or Line2D)
            "label": {},            # sid -> Text. Id label for the ship.
            "status_text": None,    # Text (global)
            "dynamic_list": []      # flat list of dynamic artists (for returning)
        }

        # Compute each artist
        for k, sid in enumerate(ship_ids):
            color = palette[k % len(palette)]

            # Trail line (empty initially)
            trail_line, = ax_map.plot([], [], lw=2.0, alpha=0.9, color=color)
            artists["trail"][sid] = trail_line
            artists["dynamic_list"].append(trail_line)

            # Ship outline as Polygon patch (empty placeholder vertices)
            # Use a small dummy triangle; you will replace it in update() via set_xy()
            dummy_xy = np.array([[0.0, 0.0],
                                [0.0, 0.0],
                                [0.0, 0.0]])
            poly = patches.Polygon(dummy_xy, closed=True, fill=False, lw=1.5, ec=color, alpha=0.8)
            ax_map.add_patch(poly)
            artists["outline"][sid] = poly
            artists["dynamic_list"].append(poly)

            # Optional label near ship
            if with_labels:
                txt = ax_map.text(0.0, 0.0, sid, fontsize=9, color=color)
                artists["label"][sid] = txt
                artists["dynamic_list"].append(txt)

        # Status bar text (global)
        status_txt = ax_status.text(
            0.01, 0.5, "", transform=ax_status.transAxes,
            va="center", ha="left", fontsize=10
        )
        artists["status_text"] = status_txt
        artists["dynamic_list"].append(status_txt)

        return artists
    
    
    def _update_dynamic(self, i, ship_ids, artists, data, precomputed_outlines=None, trail_len=None):
        # status message (customize later)
        artists["status_text"].set_text(f"time={i*self.stepSize/1e9} s | frame={i}")

        for sid in ship_ids:
            east  = data[sid]["east"]
            north = data[sid]["north"]

            # trail segment
            j0 = 0 if trail_len is None else max(0, i - int(trail_len))
            artists["trail"][sid].set_data(east[j0:i+1], north[j0:i+1])

            # outline: precomputed
            if precomputed_outlines is not None:
                artists["outline"][sid].set_xy(precomputed_outlines[sid][i])
            else:
                # fallback: compute on the fly (kept as backup)
                yaw = data[sid]["yaw"]
                draw = getattr(self, f"{sid}_draw")
                x_local, y_local = draw.local_coords()
                x_rot, y_rot = draw.rotate_coords(x_local, y_local, yaw[i])
                x_tr,  y_tr  = draw.translate_coords(x_rot, y_rot, north[i], east[i])
                xy = np.column_stack([y_tr, x_tr])
                artists["outline"][sid].set_xy(xy)

            # label
            if sid in artists["label"]:
                artists["label"][sid].set_position((east[i], north[i]))

        return artists["dynamic_list"]
    
    
    def AnimateFleetTrajectory(
        self,
        ship_ids=None,
        fig_width=10.0,
        margin_frac=0.08,
        equal_aspect=True,
        interval_ms=20,
        frame_step=1,
        trail_len=300,
        plot_routes=True,
        plot_waypoints=True,
        plot_roa=True,
        with_labels=True,
        precompute_outlines=True,
        save_path=None,
        writer_fps=20,
        show=True,
        block=True,
        palette=None,
        blit=False
    ):
        ship_ids = self._resolve_ship_ids(ship_ids)
        if len(ship_ids) == 0:
            raise ValueError("No valid ship ids to animate.")

        data, n_frames = self._prepare_playback_data(ship_ids)
        bounds = self._compute_bounds_from_playback_data(data, ship_ids)

        fig, ax_map, ax_status = self._init_anim_figures(
            fig_width=fig_width,
            equal_aspect=equal_aspect,
            margin_frac=margin_frac,
            bounds=bounds
        )

        static_artists = self._draw_static(
            ax_map=ax_map,
            ship_ids=ship_ids,
            plot_routes=plot_routes,
            plot_waypoints=plot_waypoints,
            plot_roa=plot_roa,
            palette=palette
        )

        if plot_routes:
            ax_map.legend(loc="upper right")

        artists = self._init_dynamic_artists(
            ax_map=ax_map,
            ax_status=ax_status,
            ship_ids=ship_ids,
            palette=palette,
            with_labels=with_labels
        )

        outlines = None
        if precompute_outlines:
            outlines = self._precompute_outlines(data, ship_ids, n_frames)

        # Frame skipping
        frame_step = max(1, int(frame_step))
        frames = range(0, n_frames, frame_step)

        # Interval in ms for GUI playback
        interval_ms = interval_ms

        def update(i):
            return self._update_dynamic(
                i=i,
                ship_ids=ship_ids,
                artists=artists,
                data=data,
                precomputed_outlines=outlines,
                trail_len=trail_len
            )

        self.ani = FuncAnimation(
            fig,
            update,
            repeat=False,
            frames=frames,
            interval=interval_ms,
            blit=blit
        )

        if save_path:
            writer = FFMpegWriter(fps=writer_fps)
            self.ani.save(save_path, writer=writer)

        if show:
            plt.show(block=block)

        return fig, ax_map, ax_status, self.ani
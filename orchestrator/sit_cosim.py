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
from orchestrator.utils import ShipDraw, compile_ship_params, get_ship_route_path_from_group, get_map_path
from map_route_plotter.prepare_map_route import get_gdf_from_gpkg, get_polygon_from_gdf, load_waypoints

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
                 config         : dict,
                 ROOT           : Path,
                 spawn_requests : dict=None,
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
        
        # If we have a custom spawn_request, alter the ship_configs
        if spawn_requests is not None:
            ship_configs = self.AddShipSpawn(spawn_requests=spawn_requests,
                                             ROOT=ROOT,
                                             simu_config=simu_config,
                                             ship_configs=ship_configs)
        
        # Set up the FMUs for all ship assets
        self.AddAllShips(ship_configs=ship_configs, ROOT=ROOT)
        
        # =========================
        # Set the Map (if given)
        # =========================
        # Get the map filename if exists, else None
        self.map            = simu_config.get("map", None)
        self.is_map_exists  = True if self.map is not None else False
        
        # Add the Map
        if self.is_map_exists:
            self.map_name = self.map.get("name")
            self.AddMap(ROOT=ROOT, map_filename=self.map.get("filename"))


# =============================================================================================================
# Prepare Map
# =============================================================================================================
    def AddMap(self, ROOT, map_filename):
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
            
        # Merge land geometries into a single Shapely polygon
        # (useful for spatial checks such as grounding detection)
        self.land_poly = get_polygon_from_gdf(self.land_gdf)
    
    
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
    
    
    def GetShipSpawn(self,
                     raw_route: tuple,
                     start_time: float= 0.0,
                     north: float= None,
                     east: float= None,
                     yaw_angle_deg: float= None,
                     speed_setpoints: float|list=5.0):
        """
            Converts ship spawn requests into the dictionary 
            for the ship configuration dictionary compliant 
            with `AddAllShips()` method.
        """
        ## Compile route
        north_list   = raw_route[0]
        east_list    = raw_route[1]
        if isinstance (speed_setpoints, list):
            if len(speed_setpoints) == len(north_list):
                speed_list  = speed_setpoints
            else:
                raise Exception("Speed set point length has to match with the route length!")
        else:
            speed_list      = [speed_setpoints] * len(north_list)
            speed_list[0]   = 0 # Setting initial speed as 0 when speed_setpoint is not specified
            
        route = {
            "north": north_list,
            "east": east_list,
            "speed": speed_list[1:] # First entry is the inital forward speed
        }
        
        ## Compile spawn
        if north is None:
            north_spawn = north_list[0]
        else:
            north_spawn = north
            
        if east is None:
            east_spawn  = east_list[0]
        else:
            east_spawn  = east
            
        if yaw_angle_deg is None:
            d_north             = north_list[1] - north_list[0]
            d_east              = east_list[1] - east_list[0]
            yaw_angle_rad       = np.atan2(d_east, d_north)
            yaw_angle_deg_spawn = np.rad2deg(yaw_angle_rad)
        else:
            yaw_angle_deg_spawn = yaw_angle_deg
        
        spawn = {
            "start_time": start_time,
            "north": north_spawn,
            "east": east_spawn,
            "yaw_angle_deg": yaw_angle_deg_spawn,
            "forward_speed": speed_list[0]
        }
            
        return route, spawn
    

    def AddShipSpawn(self, ROOT, spawn_requests, simu_config, ship_configs):
        """
            Adds converted spawn requests to each ship configuration dictionary. 
            This allows ships to be dynamically inserted into the simulation.
        """
        for i, ship_config in enumerate(ship_configs):
            # Get the ship id
            ship_id         = ship_config["id"]
            
            # Break down the spawn request
            spawn_request   = spawn_requests[ship_id]
            start_time      = spawn_request.get("start_time", 0.0)
            north           = spawn_request.get("north", None)
            east            = spawn_request.get("east", None)
            yaw_angle_deg   = spawn_request.get("yaw_angle_deg", None)
            speed_setpoints = spawn_request.get("speed_setpoints", 5.0)
            
            # Get the ship's unprocessed route
            # If route is not belong to any group, access it from data/route/ directly
            map             = simu_config.get("map", None)
            group           = None if map is None else map.get("group", None)
            route_filename  = ship_config.get("route_filename")
            
            if route_filename is not None:
                ship_route_path = get_ship_route_path_from_group(ROOT=ROOT, 
                                                                group=group,
                                                                route_filename=route_filename)
                raw_route = load_waypoints(ship_route_path)
            else:
                raw_route = (spawn_request.get("north_route"), spawn_request.get("east_route"))
            
            # Prepare the route and spawn request
            route, spawn = self.GetShipSpawn(raw_route=raw_route,
                                             start_time=start_time,
                                             north=north,
                                             east=east,
                                             yaw_angle_deg=yaw_angle_deg,
                                             speed_setpoints=speed_setpoints)
            
            # Assign the prepared route and spawn requests to the ship configs
            ship_configs[i]["spawn"] = spawn
            ship_configs[i]["route"] = route
        
        return ship_configs
    
        
    def AddAllShips(self,
                    ship_configs: list,
                    ROOT: Path,):
        '''
            Add all ships based on its configuration and auto solve FMU
            connections between each ship entity. Also help adjusts for 
            the Collision Avoidance FMU connection with the other relevant
            FMUs.

            This includes connections required for:
            -   **Collision Avoidance FMUs**
            -   inter-ship communication
        '''
        
        # Store ship configs as the class attribute
        self.ship_configs = ship_configs
        
        for ship_config in ship_configs:
            prefix              = ship_config.get("id")
            SHIP_BLOCKS         = ship_config.get("SHIP_BLOCKS")
            SHIP_CONNECTIONS    = ship_config.get("SHIP_CONNECTIONS")
            SHIP_OBSERVERS      = ship_config.get("SHIP_OBSERVERS")
            fmu_params          = compile_ship_params(ship_config)
            enable_colav        = ship_config.get("enable_colav")
            
            # Spawn the ships
            spawn               = ship_config["spawn"]
            fmu_params          = self.spawn_ship(ship_id=prefix, spawn=spawn, fmu_params=fmu_params)
            
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
# Simulator Step Up and Reset
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
            
            The masking protocol are, until the delayed start period end:
                - For any positional input, keep the input value constant
                - For any actuation input (thorttle, speed, shaft_speed), hold the input at zero
                - Specifically for COLAV, target ship positional inputs are set to a VERY far away
                point. This is done to prevent COLAV activation.
        """
        ## Get the base FMU name
        in_slave_base_name = self.get_fmu_base_name(in_slave)
        ship_model_name = self.ship_slave(ship_id, "SHIP_MODEL")

        ## Positional input
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

        ## Actuation Input
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

        ## Pattern-based masking for COLAV targets
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
        """
            This method handles the manipulation between FMUs.
            This also supports delayed start feature. Delayed start enables
            target ship to start simulating at time later than 0.
            
            Cosimulation only knows FMUs as it is, and do not contain information
            about which ship FMUs belong to. Thus additional protocol is needed
            to delayed start a ship simulator.
            
            Each ship has unique FMUs coded with three inital letters and "__":
            - OS0__[FMU_NAME] -> Own Ship
            - TS*__[FMU_NAME] -> Target Ship {1, 2, 3, ...}
            When a target ship start is delayed, we are masking the input value
            to each associated FMU so that when the FMUs are step up, the FMU
            does not evolve in a meaningful way. The goal is to make the ship 
            stayed still until the simulation time reach delayed start marker.
            
            This is achievable through protocol explained in get_masked_input_val_for_delayed_ship()
        """
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
        ## Get the reach end point and collision flag for all assets
        rew_flags       = {ship_config["id"]: False for ship_config in self.ship_configs}
        collision_flags = {ship_config["id"]: False for ship_config in self.ship_configs}
        
        for ship_config in self.ship_configs:
            ship_id = ship_config.get("id")
            spawn   = ship_config["spawn"] if "spawn" in list(ship_config.keys()) else None
            
            ## Update ship active status
            if spawn is not None:
                self.update_ship_delayed_status(ship_id, spawn)
            
            ## Update reach end point status
            rew_flag  = self.update_ship_reach_end_point_status(ship_id)
            rew_flags[ship_id] = rew_flag
            
            if ship_config.get("enable_colav"):
                ## Update colav active flag
                colav_active    = self.update_colav_active_flag(ship_id)
                
                ## Update collision status
                collision_flag  = self.update_collision_status(ship_id)
                collision_flags[ship_id] = collision_flag
                
                ## Update message
                if colav_active and (not collision_flag) and (not self.print_col_msg):
                    print(f"{ship_id}_COLAV is active!")
                    self.print_col_msg = True
                elif (not colav_active) and self.print_col_msg:
                    print(f"{ship_id}_COLAV is deactivated!")
                    self.print_col_msg = False
                elif collision_flag:
                    print(f"{ship_id} collides!")
        
        own_ship_reaches_end_waypoint   = rew_flags[self.ship_configs[0]["id"]]
        all_ship_reaches_end_waypoint   = np.all([rew_flags[ship_id] for ship_id in list(rew_flags.keys())])
        any_ship_collides               = np.any([collision_flags[ship_id] for ship_id in list(collision_flags.keys())])
        
        ## Conclude the stop flag
        self.stop = own_ship_reaches_end_waypoint or (all_ship_reaches_end_waypoint or any_ship_collides)
        

    def step(self):
        # Simulate
        self.CoSimManipulate()
        self.SetInputFromExternal()
        self.PreSolverFunctionCall()
        self.execution.step()
        self.PostSolverFunctionCall()
        
    
    def reset(self, ship_id="*", slave_name="*"):
        """
            [UNTESTED]
            Method to reset the FMU(s) to its/their initial state.
            
            Parameter
            ---------
            ship_id : str
                - Use to reset the target FMU with ship_id tag. 
                - If ship_id is set to "*", reset the target FMUs for all ships asset given the ship has the FMU
            slave_name: str
                - Use to set the target FMU to reset.
                - If slave_name is set to "*", reset all of the FMUs associated with the ship_id
                - If both ship_id and slave_name is set to "*", reset all of the FMUs in the simulator
        """
        def get_target_ship_configs():
            if ship_id == "*":
                return self.ship_configs

            for ship_config in self.ship_configs:
                if ship_config.get("id") == ship_id:
                    return [ship_config]

            raise ValueError(f"Unknown ship_id: {ship_id}")

        for ship_config in get_target_ship_configs():
            prefix = ship_config.get("id")
            fmu_params = compile_ship_params(ship_config)

            if slave_name == "*":
                for block, params in fmu_params.items():
                    self.SetInitialValues(
                        slaveName=self.ship_slave(prefix, block),
                        params=params
                    )
            else:
                params = fmu_params.get(slave_name)
                if params is None:
                    raise ValueError(
                        f"Unknown slave_name '{slave_name}' for ship_id '{prefix}'"
                    )

                self.SetInitialValues(
                    slaveName=self.ship_slave(prefix, slave_name),
                    params=params
                )
        return
    
    
    def Simulate(self):
        """
            Simulate the model by running step until the termination condition is triggered
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
# Plot Style for Static Plot and Animation
# =============================================================================================================
    def get_plot_style(self, mode="quick"):
        """
            Plotting conventions for both static plot and animation
        """
        if mode == "paper":
            return {
                "own_lw": 2.4,
                "route_lw": 1.8,
                "ship_lw": 2.0,
                "title_fs": 12,
                "label_fs": 11,
                "tick_fs": 10,
                "legend_fs": 9,
                "grid_alpha": 1.0,
                "roa_alpha": 0.25,
                "dpi": 500,
                "waypoint_s": 30,
                "ship_label_fs": 10,
                "status_fs": 10,
                "startend_dy": 500.0,
            }
        elif mode == "quick":
            return {
                "own_lw": 1.2,
                "route_lw": 1.0,
                "ship_lw": 1.6,
                "title_fs": 9,
                "label_fs": 8,
                "tick_fs": 7,
                "legend_fs": 7,
                "grid_alpha": 0.8,
                "roa_alpha": 0.15,
                "dpi": 110,
                "waypoint_s": 20,
                "ship_label_fs": 9,
                "status_fs": 10,
                "startend_dy": 500.0,
            }
        else:
            raise ValueError("mode must be 'quick' or 'paper'")


# =============================================================================================================
# Simulation Result Plot
# =============================================================================================================
    def JoinPlotTimeSeries(
            self,
            key_group_list,
            create_title: bool = False,
            legend: bool = True,
            show_instance_name: bool = False,
            show_separately: bool = False,
            show: bool = True,
            mode: str = "quick"
        ):
        """
        Plot multiple observer time series. Also allows joint plot for multiple
        timeseries. Ideally, the joined timeseries variables required to have the 
        same unit.

        Parameters
        ----------
        key_group_list : list[list[str]]
            List of key groups. Each group is plotted in one figure.
        create_title : bool
            If True, include the instance name in the figure title.
        legend : bool
            Show legend for each plotted signal.
        show_instance_name : bool
            Prefix labels with simulation instance name.
        show_separately : bool
            If True, each group is shown immediately.
        show : bool
            If False, plots are created but not displayed.
        mode : {"quick", "paper"}
            Plot styling mode.
            - quick: lightweight plotting for debugging
            - paper: high-quality publication style
        """

        # Plot style
        style = self.get_plot_style(mode)

        fig_w = 9
        fig_h = 7

        for key_group in key_group_list:

            struct_time_points = []
            struct_step_number = []
            struct_samples     = []
            struct_labels      = []

            # Gather data
            for key in key_group:

                time_points, step_number, samples = self.GetObserverTimeSeries(key)

                struct_time_points.append(time_points)
                struct_step_number.append(step_number)
                struct_samples.append(samples)

                label = str(key)
                if show_instance_name:
                    label = f"{self.instanceName}: {key}"

                struct_labels.append(label)

            # Create figure
            plt.figure(figsize=(fig_w, fig_h), dpi=style["dpi"])

            for i in range(len(key_group)):
                plt.plot(
                    struct_time_points[i],
                    struct_samples[i],
                    linewidth=style["own_lw"],
                    label=struct_labels[i]
                )

            # Axes formatting
            plt.grid(alpha=style["grid_alpha"])

            plt.xticks(fontsize=style["tick_fs"])
            plt.yticks(fontsize=style["tick_fs"])

            plt.xlabel("Time [s]", fontsize=style["label_fs"])
            plt.ylabel(self.observer_time_series_label[key_group[0]], fontsize=style["label_fs"])

            # Legend
            if legend:
                plt.legend(fontsize=style["legend_fs"])

            # Title
            if create_title:
                plt.title(
                    f"Time series from co-simulation instance \"{self.instanceName}\"",
                    fontsize=style["title_fs"]
                )

            plt.tight_layout()

            # Show control
            if show_separately and show:
                plt.show()

        if not show_separately and show:
            plt.show()


# =============================================================================================================
# Static Plot
# =============================================================================================================
    def get_ship_timeseries(self, ship_id: str, var: str):
        key = f"{ship_id}.{var}"
        t, step, samples = self.GetObserverTimeSeries(key)
        return np.asarray(t), np.asarray(step), np.asarray(samples)
    
    
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

    
    def PlotFleetTrajectory(
        self,
        ship_ids=None,
        show=True,
        block=True,
        mode="quick",
        fig_width=10.0,
        every_n=25,
        margin_frac=0.08,
        equal_aspect=True,
        save_path=None,
        plot_routes=True,
        plot_waypoints=True,
        plot_outlines=True,
        palette=None,
        ship_scale=1.0,
    ):
        """
            Plot the fleet trajectories for one or more ships, with optional support
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
            
            every_n : int, optional
                Number of timestep to skip between outline draws.

            margin_frac : float, optional
                Margin fraction used only in the no-map case.
                When no map is present, trajectory bounds are computed from ship motion,
                then expanded by this fraction of the total span.
                Example:
                - `margin_frac=0.08` means add 8% padding around the data
                Ignored when map exists, because map bounds are used directly.

            equal_aspect : bool, optional
                Whether to enforce equal scaling on x and y axes.

            plot_routes : bool, optional
                Whether to draw each ship's planned route from `self.ship_configs`.
                
            plot_waypoints : bool, optional
                Whether to draw waypoint markers from the configured route.
        """
        # Pick ships
        if ship_ids is None:
            ship_ids = [sc.get("id") for sc in self.ship_configs]
        ship_ids = [sid for sid in ship_ids if sid is not None]

        # Plot configurations
        style           = self.get_plot_style(mode)
        own_lw          = style.get("own_lw")
        route_lw        = style.get("route_lw")
        ship_lw         = style.get("ship_lw")
        title_fs        = style.get("title_fs")
        label_fs        = style.get("label_fs")
        tick_fs         = style.get("tick_fs")
        legend_fs       = style.get("legend_fs")
        grid_alpha      = style.get("grid_alpha")
        roa_alpha       = style.get("roa_alpha")
        dpi             = style.get("dpi")
        waypoint_s      = style.get("waypoint_s")
        startend_dy     = style.get("startend_dy")

        every_n = max(1, int(every_n))

        ## Figure
        fig, ax = plt.subplots(figsize=(fig_width, fig_width), dpi=dpi)
        
        ## Plot map if exists
        if self.is_map_exists:
            # Set the map static artists
            (frame_gdf, _, _, _, _, _, _, _, _, _, _) = self.set_map_static_artist(ax=ax)
            
            # Get the aspect
            aspect, minx, miny, maxx, maxy = self.set_map_static_aspect(frame_gdf)
            
            # Set the fig height based on the aspect
            fig_height = fig_width / aspect
        
            # Resize the figure
            fig.set_size_inches(fig_width, fig_height)

        # simple palette
        if palette is None:
            palette = ["#0c3c78", "#d90808", "#2a9d8f", "#f4a261", "#6a4c93", "#264653"]

        # track global extents (route or traj)
        all_x, all_y = [], []

        # per-ship plotting
        for k, sid in enumerate(ship_ids):
            color = palette[k % len(palette)]

            # time series 
            _, _, north = self.get_ship_timeseries(sid, "north")
            _, _, east  = self.get_ship_timeseries(sid, "east")
            _, _, yaw   = self.get_ship_timeseries(sid, "yaw_angle_rad")
            
            n = min(len(north), len(east), len(yaw))
            north, east, yaw = north[:n], east[:n], yaw[:n]

            all_x.append(east)
            all_y.append(north)
            
            # trajectory
            ax.plot(east, north, lw=own_lw, color=color, label=f"{sid} trajectory")

            # route per ship (from config) 
            if plot_routes:
                cfg = next((sc for sc in self.ship_configs if sc.get("id") == sid), None)
                if cfg and "route" in cfg:
                    rN = cfg["route"].get("north", [])
                    rE = cfg["route"].get("east", [])
                    if len(rN) and len(rE):
                        ax.plot(rE, rN, lw=route_lw, ls="--", color=color, alpha=0.7,
                                label=f"{sid} route")
                        if plot_waypoints:
                            ax.scatter(rE, rN,
                                    s=waypoint_s if mode == "quick" else 30,
                                    edgecolors="white", color=color)
                            
                            ax.text(rE[0], rN[0]+startend_dy, " START", fontsize=label_fs, weight="bold", va="bottom", ha="center", zorder=12)
                            ax.text(rE[-1], rN[-1]+startend_dy, " END", fontsize=label_fs, weight="bold", va="bottom", ha="center", zorder=12)

                            # RoA circles
                            ra = cfg["fmu_params"].get("MISSION_MANAGER", {}).get("ra", None)
                            if ra is not None:
                                for n_wp, e_wp in zip(rN[1:], rE[1:]):
                                    circ = patches.Circle(
                                        (e_wp, n_wp),
                                        radius=ra,
                                        fill=True,
                                        color=color,
                                        alpha=roa_alpha
                                    )
                                    ax.add_patch(circ)

            # ship outlines 
            if plot_outlines:
                print("Drawing ship outlines ...")
                idx = np.arange(0, n, every_n)
                draw_attr = f"{sid}_draw"
                draw = getattr(self, draw_attr)
                for i in idx[::1]:  # extra subsample for speed
                    x, y = draw.local_coords(scale=ship_scale)
                    x_ned, y_ned = draw.rotate_coords(x, y, yaw[i])
                    x_tr,  y_tr  = draw.translate_coords(x_ned, y_ned, north[i], east[i])
                    ax.plot(y_tr, x_tr, lw=ship_lw, color=color, alpha=0.6)

        # Set figure boundary and add scalebar
        if self.is_map_exists:
            # Set figure boundaries
            ax.set_xlim(minx, maxx)
            ax.set_ylim(miny, maxy)

        else:
            X = np.concatenate(all_x) if len(all_x) else np.array([0.0, 1.0])
            Y = np.concatenate(all_y) if len(all_y) else np.array([0.0, 1.0])

            x_min, x_max = float(np.min(X)), float(np.max(X))
            y_min, y_max = float(np.min(Y)), float(np.max(Y))

            dx = max(1e-9, x_max - x_min)
            dy = max(1e-9, y_max - y_min)
            
            ax.set_xlim(x_min - margin_frac * dx, x_max + margin_frac * dx)
            ax.set_ylim(y_min - margin_frac * dy, y_max + margin_frac * dy)
            
        # Add scalebar on map
        self.add_scalebar(ax=ax, length_m=None, label_fs=style["label_fs"])

        # Styling
        title = f"Fleet trajectories on {self.map_name}" if self.is_map_exists else "Fleet trajectories"
        ax.set_title(title, fontsize=title_fs, pad=4)
        ax.set_xlabel("East position (m)", fontsize=label_fs)
        ax.set_ylabel("North position (m)", fontsize=label_fs)

        ax.tick_params(axis="both", which="major", labelsize=tick_fs, length=3)
        ax.grid(True, color="0.82", linestyle="--", linewidth=0.5, alpha=grid_alpha)

        leg = ax.legend(fontsize=legend_fs, frameon=True, framealpha=0.75,
                        borderpad=0.4, handlelength=2.2, loc="upper left")
        leg.get_frame().set_linewidth(0.6)

        ax.ticklabel_format(style='sci', axis='both', scilimits=(0,0))
        ax.xaxis.get_offset_text().set_fontsize(tick_fs-1)
        ax.yaxis.get_offset_text().set_fontsize(tick_fs-1)
        
        for spine in ax.spines.values():
            spine.set_linewidth(0.8)
            spine.set_color("0.35")

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
    def resolve_ship_ids(self, ship_ids):
        if ship_ids is None:
            ship_ids = [sc.get("id") for sc in self.ship_configs]

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


    def prepare_playback_data(self, ship_ids):
        data = {}

        for sid in ship_ids:
            _, _, north = self.get_ship_timeseries(sid, "north")
            _, _, east  = self.get_ship_timeseries(sid, "east")
            _, _, yaw   = self.get_ship_timeseries(sid, "yaw_angle_rad")

            n = min(len(north), len(east), len(yaw))
            north, east, yaw = north[:n], east[:n], yaw[:n]

            data[sid] = {
                "north": np.asarray(north),
                "east":  np.asarray(east),
                "yaw":   np.asarray(yaw),
                "N":     n,
            }

        n_frames = min(data[sid]["N"] for sid in ship_ids)
        return data, n_frames


    def compute_bounds_from_playback_data(self, data, ship_ids):
        all_e = []
        all_n = []

        for sid in ship_ids:
            all_e.append(np.asarray(data[sid]["east"]))
            all_n.append(np.asarray(data[sid]["north"]))

        E = np.concatenate(all_e) if all_e else np.array([0.0, 1.0])
        N = np.concatenate(all_n) if all_n else np.array([0.0, 1.0])

        x_min, x_max = float(np.min(E)), float(np.max(E))
        y_min, y_max = float(np.min(N)), float(np.max(N))
        return x_min, x_max, y_min, y_max


    def precompute_outlines(self, data, ship_ids, n_frames, ship_scale=1.0):
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

            x_local, y_local = draw.local_coords(scale=ship_scale)

            xy_frames = []
            for i in range(n_frames):
                x_rot, y_rot = draw.rotate_coords(x_local, y_local, yaw[i])
                x_tr, y_tr   = draw.translate_coords(x_rot, y_rot, north[i], east[i])

                # final plotting convention: x = east, y = north
                xy = np.column_stack([y_tr, x_tr])
                xy_frames.append(xy)

            outlines[sid] = xy_frames

        return outlines


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
                dx = x_max - x_min
                dy = y_max - y_min

                min_span = 1.0
                if dx < min_span:
                    cx = 0.5 * (x_min + x_max)
                    x_min, x_max = cx - 0.5 * min_span, cx + 0.5 * min_span
                    dx = x_max - x_min

                if dy < min_span:
                    cy = 0.5 * (y_min + y_max)
                    y_min, y_max = cy - 0.5 * min_span, cy + 0.5 * min_span
                    dy = y_max - y_min

                ax_map.set_xlim(x_min - margin_frac * dx, x_max + margin_frac * dx)
                ax_map.set_ylim(y_min - margin_frac * dy, y_max + margin_frac * dy)
        
        # Add scalebar on map
        self.add_scalebar(ax=ax_map, length_m=None, label_fs=style["label_fs"])

        ## Common styling
        title = f"Fleet trajectories on {self.map_name}" if self.is_map_exists else "Fleet trajectories"
        ax_map.set_title(title, fontsize=style["title_fs"], pad=4)
        ax_map.set_xlabel("East position (m)", fontsize=style["label_fs"])
        ax_map.set_ylabel("North position (m)", fontsize=style["label_fs"])

        ax_map.tick_params(axis="both", which="major", labelsize=style["tick_fs"], length=3)
        ax_map.grid(True, color="0.82", linestyle="--", linewidth=0.5, alpha=style["grid_alpha"])

        ax_map.ticklabel_format(style="sci", axis="both", scilimits=(0, 0))
        ax_map.xaxis.get_offset_text().set_fontsize(style["tick_fs"] - 1)
        ax_map.yaxis.get_offset_text().set_fontsize(style["tick_fs"] - 1)

        for spine in ax_map.spines.values():
            spine.set_linewidth(0.8)
            spine.set_color("0.35")

        if equal_aspect:
            ax_map.set_aspect("equal", adjustable="box")

        return fig, ax_map, ax_status


    def draw_static(
        self,
        ax_map,
        ship_ids,
        mode="quick",
        plot_routes=True,
        plot_waypoints=True,
        plot_roa=True,
        plot_start_end=True,
        palette=None,
    ):
        """
        Draw static route, waypoint, START/END, and ROA objects.
        Style follows PlotFleetTrajectory.
        """
        style = self.get_plot_style(mode)

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
            rE = route.get("east", [])
            m = min(len(rN), len(rE))
            if m == 0:
                continue

            rN = np.asarray(rN[:m])
            rE = np.asarray(rE[:m])

            if plot_routes:
                line, = ax_map.plot(
                    rE, rN,
                    lw=style["route_lw"],
                    ls="--",
                    alpha=0.7,
                    color=color,
                    label=f"{sid} route",
                    zorder=2
                )
                static_artists.append(line)

            if plot_waypoints:
                wp = ax_map.scatter(
                    rE, rN,
                    s=style["waypoint_s"],
                    edgecolors="white",
                    color=color,
                    zorder=3
                )
                static_artists.append(wp)

            if plot_start_end and m >= 1:
                txt0 = ax_map.text(
                    rE[0], rN[0] + style["startend_dy"],
                    " START",
                    fontsize=style["label_fs"],
                    weight="bold",
                    va="bottom",
                    ha="center",
                    zorder=12
                )
                static_artists.append(txt0)

                txt1 = ax_map.text(
                    rE[-1], rN[-1] + style["startend_dy"],
                    " END",
                    fontsize=style["label_fs"],
                    weight="bold",
                    va="bottom",
                    ha="center",
                    zorder=12
                )
                static_artists.append(txt1)

            if plot_roa:
                fmu_params = cfg.get("fmu_params", {})
                mm = fmu_params.get("MISSION_MANAGER", {})
                ra = mm.get("ra", None)

                if ra is not None and ra > 0:
                    for n_wp, e_wp in zip(rN[1:], rE[1:]):
                        circ = patches.Circle(
                            (e_wp, n_wp),
                            radius=ra,
                            fill=True,
                            color=color,
                            alpha=style["roa_alpha"],
                            zorder=1
                        )
                        ax_map.add_patch(circ)
                        static_artists.append(circ)

        return static_artists


    def init_dynamic_artists(self, ax_map, ax_status, ship_ids, mode="quick", palette=None, with_labels=True):
        """
        Dynamic artists: trajectory trail, ship outline, ship id label, status text.
        """
        style = self.get_plot_style(mode)

        if palette is None:
            palette = ["#0c3c78", "#d90808", "#2a9d8f", "#f4a261", "#6a4c93", "#264653"]

        artists = {
            "trail": {},
            "outline": {},
            "label": {},
            "status_text": None,
            "dynamic_list": []
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

        status_txt = ax_status.text(
            0.01, 0.5, "",
            transform=ax_status.transAxes,
            va="center",
            ha="left",
            fontsize=style["status_fs"]
        )
        artists["status_text"] = status_txt
        artists["dynamic_list"].append(status_txt)

        return artists


    def update_dynamic(self, i, ship_ids, artists, data, precomputed_outlines=None, trail_len=None, ship_scale=1.0):
        artists["status_text"].set_text(f"time = {i * self.stepSize / 1e9:.2f} s   |   frame = {i}")

        for sid in ship_ids:
            east  = data[sid]["east"]
            north = data[sid]["north"]

            # Trail
            j0 = 0 if trail_len is None else max(0, i - int(trail_len))
            artists["trail"][sid].set_data(east[j0:i+1], north[j0:i+1])

            # Outline
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

            # Ship id text
            if sid in artists["label"]:
                artists["label"][sid].set_position((east[i], north[i]))

        return artists["dynamic_list"]


    def AnimateFleetTrajectory(
        self,
        ship_ids=None,
        show=True,
        block=True,
        mode="quick",
        fig_width=10.0,
        margin_frac=0.08,
        equal_aspect=True,
        interval_ms=20,
        frame_step=1,
        trail_len=300,
        plot_routes=True,
        plot_waypoints=True,
        plot_roa=True,
        plot_start_end=True,
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
        bounds = None if self.is_map_exists else self.compute_bounds_from_playback_data(data, ship_ids)

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

        artists = self.init_dynamic_artists(
            ax_map=ax_map,
            ax_status=ax_status,
            ship_ids=ship_ids,
            mode=mode,
            palette=palette,
            with_labels=with_labels
        )

        outlines = None
        if precompute_ship_outlines:
            outlines = self.precompute_outlines(data, ship_ids, n_frames, ship_scale)

        frame_step = max(1, int(frame_step))
        frames = range(0, n_frames, frame_step)

        def update(i):
            return self.update_dynamic(
                i=i,
                ship_ids=ship_ids,
                artists=artists,
                data=data,
                precomputed_outlines=outlines,
                trail_len=trail_len,
                ship_scale=ship_scale
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
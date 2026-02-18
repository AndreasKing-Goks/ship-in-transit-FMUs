"""
This module provides utilities class for the Co-simulation
"""

import numpy as np


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
    
    
def ship_snap_shot(north, east, yaw_angle):
    draw = ShipDraw()
    x, y = draw.local_coords()
    x_ned, y_ned = draw.rotate_coords(x, y, yaw_angle)
    x_ned_trans, y_ned_trans = draw.translate_coords(x_ned, y_ned, north, east)
    
    return x_ned_trans, y_ned_trans


def compile_ship_params(ship_cfg: dict) -> dict:
    route = ship_cfg["route"]
    north = route["north"]
    east  = route["east"]
    speed = route["speed"]

    if len(north) < 2 or len(east) < 2:
        raise ValueError("Route must have at least 2 points to compute initial yaw.")

    # ---- Derived / precomputed values from route ----
    d_north = north[1] - north[0]
    d_east  = east[1]  - east[0]
    initial_yaw = float(np.atan2(d_east, d_north))

    initial_north = float(north[0])
    initial_east  = float(east[0])

    # ---- Mission Manager params ----
    mm = dict(ship_cfg["fmu_params"].get("mission_manager", {}))  # ra, max_inter_wp, etc.
    mm["wp_start_north"] = float(north[0])
    mm["wp_start_east"]  = float(east[0])
    mm["wp_start_speed"] = float(speed[0])
    mm["wp_end_north"]   = float(north[-1])
    mm["wp_end_east"]    = float(east[-1])
    mm["wp_end_speed"]   = float(speed[-1])

    # Intermediate waypoints: points 1..-2
    iw_north = north[1:-1]
    iw_east  = east[1:-1]
    iw_speed = speed[1:-1]

    # If you want to enforce max_inter_wp:
    max_inter = int(mm.get("max_inter_wp", len(iw_north)))
    if len(iw_north) != len(iw_east) or len(iw_north) != len(iw_speed):
        raise ValueError("Route north/east/speed lengths mismatch.")
    if len(iw_north) != max_inter:
        # either raise or allow smaller:
        raise ValueError(f"Expected {max_inter} intermediate waypoints but got {len(iw_north)}")

    for i, (n_i, e_i, s_i) in enumerate(zip(iw_north, iw_east, iw_speed), start=1):
        mm[f"wp_{i}_north"] = float(n_i)
        mm[f"wp_{i}_east"]  = float(e_i)
        mm[f"wp_{i}_speed"] = float(s_i)

    # ---- Ship Model params (base + derived) ----
    sm = dict(ship_cfg["fmu_params"]["ship_model_base"])
    sm["initial_north_position_m"] = initial_north
    sm["initial_east_position_m"]  = initial_east
    sm["initial_yaw_angle_rad"]    = initial_yaw

    # Optional: also store these for debugging
    # sm["_derived_initial_d_north"] = float(d_north)
    # sm["_derived_initial_d_east"]  = float(d_east)
    
    if ship_cfg.get("enable_colav"):
        params = {
            "MISSION_MANAGER": mm, 
            "AUTOPILOT": dict(ship_cfg["fmu_params"]["autopilot"]),
            "SHAFT_SPEED_CONTROLLER": dict(ship_cfg["fmu_params"]["shaft_speed_controller"]),
            "THROTTLE_CONTROLLER": dict(ship_cfg["fmu_params"]["throttle_controller"]),
            "MACHINERY_SYSTEM": dict(ship_cfg["fmu_params"]["machinery_system"]),
            "RUDDER": dict(ship_cfg["fmu_params"]["rudder"]),
            "SHIP_MODEL": sm,
            "COLAV": dict(ship_cfg["fmu_params"]["collision_avoidance"])
            }
    else:
        params = {
            "MISSION_MANAGER": mm, 
            "AUTOPILOT": dict(ship_cfg["fmu_params"]["autopilot"]),
            "SHAFT_SPEED_CONTROLLER": dict(ship_cfg["fmu_params"]["shaft_speed_controller"]),
            "THROTTLE_CONTROLLER": dict(ship_cfg["fmu_params"]["throttle_controller"]),
            "MACHINERY_SYSTEM": dict(ship_cfg["fmu_params"]["machinery_system"]),
            "RUDDER": dict(ship_cfg["fmu_params"]["rudder"]),
            "SHIP_MODEL": sm
            }

    # ---- Pass-through params for other FMUs ----
    return params
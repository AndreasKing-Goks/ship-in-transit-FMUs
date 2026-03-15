"""
This module provides utilities class for the Co-simulation
"""

from pathlib import Path
import re
import datetime
import uuid

import numpy as np


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
    mm = dict(ship_cfg["fmu_params"].get("MISSION_MANAGER", {}))  # ra, max_inter_wp, etc.
    mm["wp_start_north"] = float(north[0])
    mm["wp_start_east"]  = float(east[0])
    mm["wp_end_north"]   = float(north[-1])
    mm["wp_end_east"]    = float(east[-1])
    mm["wp_end_speed"]   = float(speed[-1])

    # Intermediate waypoints: points 1..-2
    iw_north = north[1:-1]
    iw_east  = east[1:-1]
    iw_speed = speed[0:-1]

    # If you want to enforce max_inter_wp:
    max_inter_wp = int(len(iw_north))
    if len(iw_north) != len(iw_east):
        raise ValueError("Route north/east lengths mismatch.")
    mm["max_inter_wp"] = max_inter_wp

    if max_inter_wp > 0:
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
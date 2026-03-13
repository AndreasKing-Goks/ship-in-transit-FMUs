import geopandas as gpd
import numpy as np
from shapely.geometry import Polygon, MultiPolygon
from pathlib import Path

# -----------------------
# 1) LOAD + PLOT THE MAP (same axes we'll reuse for the SIT overlays)
# -----------------------
def get_gdf_from_gpkg(
    gpkg_path,
    frame_layer,
    ocean_layer,
    land_layer,
    coast_layer,
    water_layer,
    waterways_layer=None,
    ferry_routes_layer=None,
    harbours_layer=None,
    bridges_layer=None,
    tss_layer=None,
    docks_layer=None,
):
    frame_gdf = gpd.read_file(gpkg_path, layer=frame_layer)
    ocean_gdf = gpd.read_file(gpkg_path, layer=ocean_layer)
    land_gdf  = gpd.read_file(gpkg_path, layer=land_layer)

    def safe_read(layer_name):
        if layer_name is None:
            return gpd.GeoDataFrame(geometry=[], crs=ocean_gdf.crs)
        try:
            return gpd.read_file(gpkg_path, layer=layer_name)
        except Exception:
            return gpd.GeoDataFrame(geometry=[], crs=ocean_gdf.crs)

    coast_gdf = safe_read(coast_layer)
    water_gdf = safe_read(water_layer)
    waterways_gdf = safe_read(waterways_layer)
    ferry_routes_gdf = safe_read(ferry_routes_layer)
    harbours_gdf = safe_read(harbours_layer)
    bridges_gdf = safe_read(bridges_layer)
    tss_gdf = safe_read(tss_layer)
    docks_gdf = safe_read(docks_layer)

    return (
        frame_gdf, ocean_gdf, land_gdf, coast_gdf, water_gdf,
        waterways_gdf, ferry_routes_gdf, harbours_gdf, bridges_gdf, tss_gdf, docks_gdf
    )

# --- Build map_data from land polygons in the SAME CRS as your route ---
def get_polygon_from_gdf(gdf):
    polys = []
    for geom in gdf.geometry:
        if geom is None:
            continue
        if isinstance(geom, Polygon):
            polys.append(list(geom.exterior.coords))
        elif isinstance(geom, MultiPolygon):
            for p in geom.geoms:
                polys.append(list(p.exterior.coords))
    return polys


# =============================================================================================================
# Get Path
# =============================================================================================================    
def load_waypoints(route_path, print_init_msg=False):
    """Route file: two columns [north, east]. Accepts str or Path or array."""
    if print_init_msg:
        print(f"Route received in load_waypoints: {route_path}")
    if isinstance(route_path, (str, Path)):
        data = np.loadtxt(route_path)
    else:
        data = route_path
    north = data[:, 0].tolist()
    east  = data[:, 1].tolist()
    return north, east


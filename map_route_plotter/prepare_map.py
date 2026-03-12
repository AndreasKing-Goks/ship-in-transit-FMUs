import geopandas as gpd
from shapely.geometry import Polygon, MultiPolygon

# -----------------------
# 1) LOAD + PLOT THE MAP (same axes we'll reuse for the SIT overlays)
# -----------------------
def get_gdf_from_gpkg(GPKG_PATH, FRAME_LAYER, OCEAN_LAYER, LAND_LAYER, COAST_LAYER, WATER_LAYER):
    # -----------------------
    # 1) LOAD + PLOT THE MAP (same axes we'll reuse for the SIT overlays)
    # -----------------------
    frame_gdf = gpd.read_file(GPKG_PATH, layer=FRAME_LAYER)
    ocean_gdf = gpd.read_file(GPKG_PATH, layer=OCEAN_LAYER)
    land_gdf  = gpd.read_file(GPKG_PATH, layer=LAND_LAYER)

    try:
        coast_gdf = gpd.read_file(GPKG_PATH, layer=COAST_LAYER)
    except Exception:
        coast_gdf = gpd.GeoDataFrame(geometry=[], crs=ocean_gdf.crs)

    try:
        water_gdf = gpd.read_file(GPKG_PATH, layer=WATER_LAYER)
    except Exception:
        water_gdf = gpd.GeoDataFrame(geometry=[], crs=ocean_gdf.crs)
        
    return frame_gdf, ocean_gdf, land_gdf, coast_gdf, water_gdf

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
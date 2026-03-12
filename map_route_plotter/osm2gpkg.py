# pip install osmnx geopandas shapely pyproj matplotlib pandas

from __future__ import annotations

from pathlib import Path
import sys
import os

## PATH HELPER (OBLIGATORY)
# project root = two levels up from this file
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import xml.etree.ElementTree as ET
from pathlib import Path

import geopandas as gpd
import matplotlib.pyplot as plt
import osmnx as ox
import pandas as pd
from shapely.geometry import box
from shapely.ops import unary_union


# =============================================================================
# CONFIG
# =============================================================================
map_name        = "oslo_fjord"

OSM_XML         = Path("map_route_plotter/osm_data") / f"{map_name}.osm"
LAND_SHP        = Path("map_route_plotter/shp_data/land-polygons-complete-4326/land_polygons.shp")
WATER_SHP       = None
OUT_GPKG        = Path("data/map") / f"{map_name}.gpkg"

# Set to None to use OSM bounds automatically
MANUAL_BOUNDS   = (10.56, 59.82, 10.76, 59.92)  # (minx, miny, maxx, maxy)


# =============================================================================
# HELPERS
# =============================================================================

def empty_gdf(crs: str | None = "EPSG:4326") -> gpd.GeoDataFrame:
    """Create an empty GeoDataFrame with a geometry column."""
    return gpd.GeoDataFrame(geometry=[], crs=crs)


def reset_output_file(path: str) -> None:
    """Delete an existing output GeoPackage."""
    if os.path.exists(path):
        os.remove(path)


def get_bounds_from_osm(osm_xml: str) -> tuple[float, float, float, float]:
    """Read the <bounds> tag from an OSM XML file."""
    root = ET.parse(osm_xml).getroot()
    bounds = root.find("bounds")
    if bounds is None:
        raise ValueError(f"No <bounds> found in {osm_xml}")
    return tuple(
        map(float, (bounds.get("minlon"), bounds.get("minlat"), bounds.get("maxlon"), bounds.get("maxlat")))
    )


def make_frame(bounds: tuple[float, float, float, float], crs: str = "EPSG:4326") -> gpd.GeoDataFrame:
    """Create a rectangular frame polygon as a GeoDataFrame."""
    minx, miny, maxx, maxy = bounds
    return gpd.GeoDataFrame(geometry=[box(minx, miny, maxx, maxy)], crs=crs)


def safe_read_and_clip(path: str | None, frame: gpd.GeoDataFrame) -> gpd.GeoDataFrame:
    """Read a vector file and clip it to the frame. Return empty if unavailable."""
    if not path:
        return empty_gdf(frame.crs)
    try:
        return gpd.clip(gpd.read_file(path), frame)
    except Exception:
        return empty_gdf(frame.crs)


def build_ocean_from_land(frame: gpd.GeoDataFrame, land: gpd.GeoDataFrame) -> gpd.GeoDataFrame:
    """Construct an ocean polygon as frame minus land."""
    if land.empty:
        return gpd.GeoDataFrame(geometry=[frame.geometry.iloc[0]], crs=frame.crs)

    land_union = unary_union(land.geometry)
    ocean_geom = frame.geometry.iloc[0].difference(land_union)
    return gpd.GeoDataFrame(geometry=[ocean_geom], crs=frame.crs)


def osm_features_from_xml(osm_xml: str, tags: dict, crs: str = "EPSG:4326") -> gpd.GeoDataFrame:
    """Safely extract OSM features from XML."""
    try:
        gdf = ox.features_from_xml(osm_xml, tags=tags)
    except Exception:
        return empty_gdf(crs)

    if gdf is None or getattr(gdf, "empty", True):
        return empty_gdf(crs)

    if gdf.crs is None:
        gdf = gdf.set_crs(crs)

    return gdf


def concat_polygon_layers(*layers: gpd.GeoDataFrame, crs: str = "EPSG:4326") -> gpd.GeoDataFrame:
    """Concatenate only polygon/multipolygon features from input layers."""
    parts = []
    for layer in layers:
        if layer is not None and not layer.empty:
            mask = layer.geometry.geom_type.isin(["Polygon", "MultiPolygon"])
            filtered = layer[mask]
            if not filtered.empty:
                parts.append(filtered)

    if not parts:
        return empty_gdf(crs)

    return gpd.GeoDataFrame(pd.concat(parts, ignore_index=True), crs=crs)


def save_layer(gdf: gpd.GeoDataFrame | None, gpkg_path: str, layer_name: str, crs: str = "EPSG:4326") -> None:
    """Save a layer to GeoPackage, always creating the layer even if empty."""
    if gdf is None:
        gdf = empty_gdf(crs)
    gdf.to_file(gpkg_path, layer=layer_name, driver="GPKG")


def reproject_or_empty(gdf: gpd.GeoDataFrame, target_crs) -> gpd.GeoDataFrame:
    """Reproject a GeoDataFrame, preserving empties."""
    if gdf.empty:
        return empty_gdf(target_crs)
    return gdf.to_crs(target_crs)


def save_projection_group(
    layers: dict[str, gpd.GeoDataFrame],
    gpkg_path: str,
    suffix: str,
    target_crs=None,
) -> dict[str, gpd.GeoDataFrame]:
    """
    Save a group of layers with suffix naming.
    If target_crs is None, save as-is.
    Returns the saved/reprojected layer dict.
    """
    out = {}
    for name, gdf in layers.items():
        out_gdf = gdf if target_crs is None else reproject_or_empty(gdf, target_crs)
        save_layer(out_gdf, gpkg_path, f"{name}_{suffix}", crs=target_crs or gdf.crs)
        out[name] = out_gdf
    return out


def save_union_layers(
    layers: dict[str, gpd.GeoDataFrame],
    gpkg_path: str,
    suffix: str = "union_utm",
) -> None:
    """Save unary-union versions of non-empty layers."""
    for name, gdf in layers.items():
        if gdf.empty:
            continue
        union_gdf = gpd.GeoDataFrame(geometry=[unary_union(gdf.geometry)], crs=gdf.crs)
        union_gdf.to_file(gpkg_path, layer=f"{name}_{suffix}", driver="GPKG")


def plot_layer_if_not_empty(
    ax,
    gpkg_path: str,
    layer_name: str,
    **plot_kwargs,
) -> None:
    """Read a layer from GPKG and plot it if not empty."""
    gdf = gpd.read_file(gpkg_path, layer=layer_name)
    if not gdf.empty:
        gdf.plot(ax=ax, **plot_kwargs)


# =============================================================================
# MAIN
# =============================================================================

def main() -> None:
    reset_output_file(OUT_GPKG)

    bounds = MANUAL_BOUNDS if MANUAL_BOUNDS is not None else get_bounds_from_osm(OSM_XML)
    frame_wgs84 = make_frame(bounds, crs="EPSG:4326")

    # -------------------------------------------------------------------------
    # Base land/ocean
    # -------------------------------------------------------------------------
    land = safe_read_and_clip(LAND_SHP, frame_wgs84)
    ocean = safe_read_and_clip(WATER_SHP, frame_wgs84)
    if ocean.empty:
        ocean = build_ocean_from_land(frame_wgs84, land)

    # -------------------------------------------------------------------------
    # OSM feature extraction
    # -------------------------------------------------------------------------
    coast = osm_features_from_xml(OSM_XML, {"natural": "coastline"})
    water_nat = osm_features_from_xml(OSM_XML, {"natural": "water"})
    water_tag = osm_features_from_xml(OSM_XML, {"water": True})
    water = concat_polygon_layers(water_nat, water_tag)

    waterways = osm_features_from_xml(OSM_XML, {"waterway": True})
    roads = osm_features_from_xml(OSM_XML, {"highway": True})

    # More features
    docks = osm_features_from_xml(OSM_XML, {"waterway": "dock"})
    harbours = osm_features_from_xml(OSM_XML, {"harbour": True})
    ferry_terms = osm_features_from_xml(OSM_XML, {"amenity": "ferry_terminal"})
    ferry_routes = osm_features_from_xml(OSM_XML, {"route": "ferry"})
    tss = osm_features_from_xml(
        OSM_XML,
        {"seamark:type": [
            "traffic_separation_scheme",
            "separation_zone",
            "separation_line",
            "roundabout",
            "precautionary_area",
        ]},
    )
    rocks_reefs = osm_features_from_xml(OSM_XML, {"seamark:type": ["rock", "rock_awash", "reef"]})
    bridges = osm_features_from_xml(OSM_XML, {"bridge": True})
    seas = osm_features_from_xml(OSM_XML, {"place": "sea"})

    layers_wgs84 = {
        "frame": frame_wgs84,
        "ocean": ocean,
        "land": land,
        "water": water,
        "waterways": waterways,
        "coast": coast,
        "roads": roads,
        "docks": docks,
        "harbours": harbours,
        "ferry_terms": ferry_terms,
        "ferry_routes": ferry_routes,
        "tss": tss,
        "rocks_reefs": rocks_reefs,
        "bridges": bridges,
        "seas": seas,
    }

    # -------------------------------------------------------------------------
    # Save WGS84
    # -------------------------------------------------------------------------
    save_projection_group(layers_wgs84, OUT_GPKG, suffix="wgs84", target_crs=None)

    # -------------------------------------------------------------------------
    # Save EPSG:3857
    # -------------------------------------------------------------------------
    layers_3857 = save_projection_group(layers_wgs84, OUT_GPKG, suffix="3857", target_crs="EPSG:3857")

    # -------------------------------------------------------------------------
    # Save UTM
    # -------------------------------------------------------------------------
    base_for_crs = land if not land.empty else ocean
    utm_crs = base_for_crs.estimate_utm_crs()
    layers_utm = save_projection_group(layers_wgs84, OUT_GPKG, suffix="utm", target_crs=utm_crs)

    # -------------------------------------------------------------------------
    # Save union layers
    # -------------------------------------------------------------------------
    save_union_layers(
        {
            "ocean": layers_utm["ocean"],
            "land": layers_utm["land"],
            "water": layers_utm["water"],
            "coast": layers_utm["coast"],
            "docks": layers_utm["docks"],
            "harbours": layers_utm["harbours"],
            "ferry_terms": layers_utm["ferry_terms"],
            "ferry_routes": layers_utm["ferry_routes"],
            "tss": layers_utm["tss"],
            "rocks_reefs": layers_utm["rocks_reefs"],
            "bridges": layers_utm["bridges"],
            "seas": layers_utm["seas"],
        },
        OUT_GPKG,
    )

    print(f"Saved fresh layers to {OUT_GPKG}")

if __name__ == "__main__":
    main()
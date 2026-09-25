import numpy as np
import matplotlib.pyplot as plt
from copernicus_weather import OpenCopernicusWeatherDataset

weather = OpenCopernicusWeatherDataset(
    wind_dataset_id="cmems_obs-wind_glo_phy_my_l4_0.125deg_PT1H",
    current_dataset_id="cmems_mod_nws_phy-uv_my_7km-2D_PT1H-i",
    min_lon=5.00,
    max_lon=7.75,
    min_lat=62.25,
    max_lat=63.25,
    start_time="2020-01-01T00:00:00",
    end_time="2020-01-31T23:00:00"
)

target_time = "2020-01-09T13:00:00"

test_1=False
# test_1=True

if test_1:
    wind_t = weather.get_wind_field(time=target_time)

    lon = wind_t["longitude"].values
    lat = wind_t["latitude"].values

    u = wind_t["eastward_wind"].values
    v = wind_t["northward_wind"].values


    LON, LAT = np.meshgrid(lon, lat)

    plt.figure(figsize=(10, 6))

    plt.quiver(LON, LAT, u, v)

    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title("Wind field - 2020-01-02 13:00")

    plt.show()

test_2=False
# test_2=True

if test_2:
    current_t = weather.get_current_field(time=target_time)

    lon = current_t["longitude"].values
    lat = current_t["latitude"].values

    u = current_t["uo"].values
    v = current_t["vo"].values


    LON, LAT = np.meshgrid(lon, lat)

    plt.figure(figsize=(10, 6))

    plt.quiver(LON, LAT, u, v) # Scale works inversely

    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title("Current field - 2020-01-02 13:00")

    plt.show()

test_3=False
# test_3=True

if test_3:
    current_t = weather.get_current_field(time=target_time)
    wind_t = weather.get_wind_field(time=target_time)

    lon_w = wind_t["longitude"].values
    lat_w = wind_t["latitude"].values

    lon_c = current_t["longitude"].values
    lat_c = current_t["latitude"].values

    u_w = wind_t["eastward_wind"].values
    v_w = wind_t["northward_wind"].values

    u_c = current_t["uo"].values
    v_c = current_t["vo"].values
    
    # print(u_c)
    # print("##")
    # print(v_c)
    
    # print(lon_w.shape)
    # print(lat_w.shape)
    
    # print(u_w.shape)
    # print(v_w.shape)
    
    LON_W, LAT_W = np.meshgrid(lon_w, lat_w)
    LON_C, LAT_C = np.meshgrid(lon_c, lat_c)

    plt.figure(figsize=(10, 6))

    plt.quiver(LON_W, LAT_W, u_w, v_w)
    plt.quiver(LON_C, LAT_C, u_c, v_c)

    plt.xlabel("Longitude")
    plt.ylabel("Latitude")

    plt.show()
    
test_4=False
test_4=True

if test_4:
    lon_stepsize = 0.125
    lat_stepsize = 0.125
    weather.get_common_grid(lon_stepsize, lat_stepsize)
    
    lon = weather.common_lon
    lat = weather.common_lat
    
    u_w_field, v_w_field = weather.get_common_wind_field(target_time)
    u_c_field, v_c_field = weather.get_common_current_field(target_time)
    
    LON, LAT = np.meshgrid(lon, lat)
    
    plt.figure(figsize=(10, 6))
    
    plt.quiver(LON, LAT, u_w_field, v_w_field)
    plt.quiver(LON, LAT, u_c_field, v_c_field)
    
    plt.show()
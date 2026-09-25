import numpy as np
import copernicusmarine

class OpenCopernicusWeatherDataset:
    def __init__(
        self,
        wind_dataset_id,
        current_dataset_id,
        min_lon,
        max_lon,
        min_lat,
        max_lat,
        start_time,
        end_time
    ):
        # Latitude-Longitude
        self.min_lon = min_lon
        self.max_lon = max_lon
        self.min_lat = min_lat
        self.max_lat = max_lat
        
        # Wind  
        self.wind = copernicusmarine.open_dataset(
            dataset_id=(wind_dataset_id),
            variables=[
                "eastward_wind",
                "northward_wind"
            ],
            minimum_longitude=min_lon,
            maximum_longitude=max_lon,
            minimum_latitude=min_lat,
            maximum_latitude=max_lat,
            start_datetime=start_time,
            end_datetime=end_time
        )
        
        # Current
        self.current = copernicusmarine.open_dataset(
            dataset_id=(current_dataset_id),
            variables=[
                "uo",
                "vo"
            ],
            minimum_longitude=min_lon,
            maximum_longitude=max_lon,
            minimum_latitude=min_lat,
            maximum_latitude=max_lat,
            start_datetime=start_time,
            end_datetime=end_time
        )
        
    def get_wind_field(self, time):
        time = np.datetime64(time)
        
        return self.wind.sel(
            time=time
        )
        
    def get_current_field(self, time):
        time = np.datetime64(time)
        
        return self.current.sel(
            time=time
        )
        
    def get_common_grid(self, lon_stepsize, lat_stepsize):
        self.common_lon = np.arange(self.min_lon, self.max_lon, lon_stepsize)
        self.common_lat = np.arange(self.min_lat, self.max_lat, lat_stepsize)
    
    def get_common_wind_field(self, time):
        wind = self.wind.interp(
            longitude=self.common_lon,
            latitude=self.common_lat,
            time=np.datetime64(time)
        ).compute()
        
        u_w = wind["eastward_wind"].values
        v_w = wind["northward_wind"].values
        
        return u_w, v_w
    
    def get_common_current_field(self, time):
        current = self.current.interp(
                longitude=self.common_lon,
                latitude=self.common_lat,
                time=np.datetime64(time)
            ).compute()
        
        u_c = current["uo"].values
        v_c = current["vo"].values
        
        return u_c, v_c
    
    # def get_common_wind_field(self, time):
    #     common_u_w_field = []
    #     common_v_w_field = []
        
    #     for lat in self.common_lat:
    #         u_w_row = []
    #         v_w_row = []
    #         for lon in self.common_lon:
    #             wind = self.wind.interp(
    #                 longitude=lon,
    #                 latitude=lat,
    #                 time=time
    #             )
                
    #             u_w_row.append(wind["eastward_wind"].item())
    #             v_w_row.append(wind["northward_wind"].item())
            
    #         common_u_w_field.append(u_w_row)
    #         common_v_w_field.append(v_w_row)
        
    #     return common_u_w_field, common_v_w_field
    
    # def get_common_current_field(self, time):
    #     common_u_c_field = []
    #     common_v_c_field = []
        
    #     for lat in self.common_lat:
    #         u_c_row = []
    #         v_c_row = []
    #         for lon in self.common_lon:
    #             current = self.current.interp(
    #                 longitude=lon,
    #                 latitude=lat,
    #                 time=time
    #             ).compute()
            
    #             u_c_row.append(current["uo"].item())
    #             v_c_row.append(current["vo"].item())
            
    #         common_u_c_field.append(u_c_row)
    #         common_v_c_field.append(v_c_row)
        
    #     return common_u_c_field, common_v_c_field
    
    def get_weather(self, lon, lat, time):
        
        # Get time
        time = np.datetime64(time)
        
        # Interpolate wind
        wind = self.wind.interp(
            longitude=lon,
            latitude=lat,
            time=time
        ).compute()
        
        # Interpolate current
        current = self.current.interp(
            longitude=lon,
            latitude=lat,
            time=time
        ).compute() # Include .compute() because current output dask, data to compute the current, not a value directly like wind
        
        return {
            "wind": np.array([
                wind["eastward_wind"].item(),
                wind["northward_wind"].item(),
            ]),
            
            "current": np.array([
                current["uo"].item(),
                current["vo"].item(),
            ]),
        }
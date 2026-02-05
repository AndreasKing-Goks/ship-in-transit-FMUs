from shapely.geometry import Polygon, Point
import matplotlib.patches as patches

import numpy as np
import matplotlib.pyplot as plt
import os

class StaticObstacle:
    ''' This class is used to define a static obstacle. It can only make
        circular obstacles. The class is instantiated with the following
        input paramters:
        - n_pos: The north coordinate of the center of the obstacle.
        - e_pos: The east coordinate of the center of the obstacle.
        - radius: The radius of the obstacle.
        
        No need for Reset method because obstacles will not change across
        the entire SAC episodes
    '''

    def __init__(self, obstacle_data, print_init_msg=False):
        
        self.obstacles = obstacle_data
        self.n_obs = []
        self.e_obs = []
        self.r_obs = []
        self.data = None
        
        self.load_obstacles(self.obstacles)
        
    def load_obstacles(self, obstacles, print_init_msg=False):
        # Load the file if the input is a string (file path)
        if isinstance(obstacles, str):
            if not os.path.exists(obstacles):
                raise FileNotFoundError(f"ERROR: File '{obstacles}' not found!")  # Check file existence
            if print_init_msg:
                print(f"Loading route file from: {obstacles}")  # Debugging
            self.data = np.loadtxt(obstacles)
        else:
            self.data = obstacles  # Assume it's already a numpy array

        if self.data.ndim == 1 and self.data.shape[0] == 3:
            # Single obstacle case, reshape to (1,3)
            self.data = self.data.reshape(1, 3)
            
        self.num_obstacles = np.shape(self.data)[0]
        
        # To avoid single case obstacles as scalar
        self.n_obs = self.data[:, 0].tolist()
        self.e_obs = self.data[:, 1].tolist()
        self.r_obs = self.data[:, 2].tolist()

    def obstacles_distance(self, n_ship, e_ship):
        ''' Returns the distance from a ship with coordinates (north, east)=
            (n_ship, e_ship), to the closest point on the perifery of the
            circular obstacle.
        '''
        list_distance = np.zeros(self.num_obstacles)
        
        for i in range(self.num_obstacles):
            rad_2 = (n_ship - self.n_obs[i]) ** 2 + (e_ship - self.e_obs[i]) ** 2
            rad = np.sqrt(abs(rad_2))
            list_distance[i] = rad - self.r_obs[i]
            
        return np.min(list_distance)
    
    def if_ship_inside_obstacles(self, n_ship, e_ship):
        ''' Checks if the ship is inside any obstacle 
        '''
        distances_squared = (n_ship - np.array(self.n_obs)) ** 2 + (e_ship - np.array(self.e_obs)) ** 2
        radii_squared = np.array(self.r_obs) ** 2

        return np.any(distances_squared <= radii_squared)  # True if inside any obstacle
    
    def if_route_inside_obstacles(self, n_route, e_route):
        ''' Checks if the sampled routes are inside any obstacle 
        '''
        distances_squared = (n_route - np.array(self.n_obs)) ** 2 + (e_route - np.array(self.e_obs)) ** 2
        radii_squared = np.array(self.r_obs) ** 2

        if np.any(distances_squared <= radii_squared):  # True if inside any obstacle
          return True
      
        return False  

    def plot_obstacle(self, ax):
        ''' This method can be used to plot the obstacle in a
            map-view.
        '''
        for i in range(self.num_obstacles):
            ax.add_patch(plt.Circle((self.e_obs[i], self.n_obs[i]), radius=self.r_obs[i], fill=True, color='grey'))
            
class PolygonObstacle:
    ''' This class defines polygonal static obstacles using Shapely.
        The polygon is defined by a list of (easting, northing) vertices.
        - No reset is needed, as obstacles are static.
    '''
    
    def __init__(self, list_of_vertices_list):
        ''' 
        Input:
        list_of_vertices_list: a list of polygons, each being a list of (e, n) tuples
        Example: [
            [(0,0), (5,0), (5,5), (0,5)],  # Square
            [(3,3), (5,3), (4,5)]          # Triangle
        ]
        '''
        self.polygons = [Polygon(verts) for verts in list_of_vertices_list]
        self.num_obstacles = len(self.polygons)
        self.map_boundaries(list_of_vertices_list)
        
    def map_boundaries(self, list_of_vertices_list):
        # Flatten the list of lists into a single list of (east, north) tuples
        map_data = list_of_vertices_list
        all_points = [point for island in map_data for point in island]
        
        # Separate into east and north components
        east_values = [p[0] for p in all_points]
        north_values = [p[1] for p in all_points]
        
        # Compute min and max
        self.min_east = min(east_values)
        self.max_east = max(east_values)
        self.min_north = min(north_values)
        self.max_north = max(north_values)
    
    def if_pos_inside_obstacles(self, n_pos, e_pos):
        ''' Check if tagged pos is inside any polygon '''
        pt = Point(e_pos, n_pos)  # x = east, y = north
        return any(poly.contains(pt) for poly in self.polygons)
    
    def if_route_inside_obstacles(self, n_route, e_route):
        ''' Check if any route point is inside any polygon '''
        for e, n in zip(e_route, n_route):
            if self.if_pos_inside_obstacles(n, e):
                return True
        return False
    
    def obstacles_distance(self, n_ship, e_ship):
        ''' Minimum distance to any polygon '''
        pt = Point(e_ship, n_ship)
        return min(poly.exterior.distance(pt) for poly in self.polygons)
    
    def plot_obstacle(self, ax):
        ''' Plot all polygonal obstacles '''
        for poly in self.polygons:
            coords = list(poly.exterior.coords)
            patch = patches.Polygon(coords, closed=True, fill=True, color='grey')
            ax.add_patch(patch)
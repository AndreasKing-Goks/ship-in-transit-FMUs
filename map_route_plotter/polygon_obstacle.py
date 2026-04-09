from shapely.geometry import Polygon, Point
import matplotlib.patches as patches

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
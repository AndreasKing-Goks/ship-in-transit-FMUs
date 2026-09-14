import numpy as np
from shapely.geometry import Polygon, Point, LineString
from shapely import STRtree
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
        # Spatial index for fast  lookup
        self.polygons = [Polygon(verts) for verts in list_of_vertices_list]
        self.num_obstacles = len(self.polygons)
        self.tree = STRtree(self.polygons)
        
        # Coastline-distance lookup
        self.boundaries = [poly.exterior for poly in self.polygons]
        self.boundary_tree = STRtree(self.boundaries)
        
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
    
    # def if_pos_inside_obstacles(self, n_pos, e_pos):
    #     ''' Check if tagged pos is inside any polygon '''
    #     pt = Point(e_pos, n_pos)  # x = east, y = north
    #     return any(poly.covers(pt) for poly in self.polygons)
    
    def if_pos_inside_obstacles(self, n_pos, e_pos):
        """
        UPGRADED VERSION
        Using spatial index STRtree by looking into polygon with the nearest location 
        from the evaluated position
        """
        
        pt = Point(e_pos, n_pos)

        # Point is tested only against geographically
        # relevant polygons selected by STRtree.
        # predicate "wihtin", only the insides
        # predicate "covered_by", includes coastline
        indices = self.tree.query(pt, predicate="covered_by")

        return len(indices) > 0
    
    def if_route_inside_obstacles(self, n_route, e_route):
        ''' Check if any route point is inside any polygon '''
        for e, n in zip(e_route, n_route):
            if self.if_pos_inside_obstacles(n, e):
                return True
        return False
    
    # def obstacles_distance(self, n_ship, e_ship):
    #     ''' Minimum distance to any polygon '''
    #     pt = Point(e_ship, n_ship)
    #     return min(poly.exterior.distance(pt) for poly in self.polygons)
    
    def obstacles_distance(self, n_ship, e_ship):
        ''' Minimum distance to any polygon '''
        pt = Point(e_ship, n_ship)
        idx = self.boundary_tree.nearest(pt)
        nearest_boundary = self.boundaries[int(idx)]

        return nearest_boundary.distance(pt)
    
    def plot_obstacle(self, ax):
        ''' Plot all polygonal obstacles '''
        for poly in self.polygons:
            coords = list(poly.exterior.coords)
            patch = patches.Polygon(coords, closed=True, fill=True, color='grey')
            ax.add_patch(patch)
            
    def get_ship_reference_points(
        n_ship,
        e_ship,
        heading_deg,
        length,
        beam,
    ):

        psi             = np.deg2rad(heading_deg)

        # Forward vector [east, north]
        forward         = np.array([np.sin(psi),np.cos(psi)])

        # Starboard vector [east, north]
        starboard       = np.array([np.cos(psi),-np.sin(psi)])
        center          = np.array([e_ship,n_ship])
        bow             = (center+ 0.5 * length * forward)
        stern           = (center- 0.5 * length * forward)
        starboard_point = (center+ 0.5 * beam * starboard)
        port_point      = (center- 0.5 * beam * starboard)

        return {
            "bow": bow,
            "stern": stern,
            "port": port_point,
            "starboard": starboard_point,
        }
    
    def directional_distance(
        self,
        origin_e,
        origin_n,
        dir_e,
        dir_n,
        max_range=5000.0,
    ):

        direction_length = np.hypot(dir_e,dir_n)
        dir_e /= direction_length
        dir_n /= direction_length

        end_e = origin_e + max_range * dir_e
        end_n = origin_n + max_range * dir_n

        # Shooting ray to one direction to see if it's intersects with the ground terrain
        ray = LineString([
            (origin_e, origin_n),
            (end_e, end_n),
        ])

        # Only coastline pieces whose bounding boxes
        # intersect the ray are considered.
        indices = self.boundary_tree.query(ray, predicate="intersects")
        if len(indices) == 0:
            return np.inf

        origin = Point(origin_e, origin_n)
        min_distance = np.inf

        for idx in indices:
            boundary = self.boundaries[int(idx)]
            intersection = ray.intersection(boundary)
            if intersection.is_empty:
                continue
            distance = origin.distance(intersection)
            min_distance = min(min_distance, distance)

        return min_distance
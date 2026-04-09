
import numpy as np
from map_route_plotter.polygon_obstacle import PolygonObstacle

def is_reaches_endpoint(route_end, pos, arrival_radius=200):
    # Unpack
    [n_test, e_test] = pos
    [n_route_end, e_route_end] = route_end
    
    # Get the relative distance between the ship and the end point
    relative_dist = np.sqrt((n_test - n_route_end)**2 + (e_test - e_route_end)**2)
    
    # Check if the ship arrive at the end point
    is_reached = relative_dist <= arrival_radius
    
    return is_reached

def is_ship_outside_horizon(map_obj : PolygonObstacle, 
                            pos, 
                            ship_length):
    ''' 
        Checks if the ship positions are outside the map horizon. 
        Map horizons are determined by the edge point of the determined route point. 
        Only works with start to end route points method (Two initial points).
    '''
    # Unpack ship position
    n_pos, e_pos = pos
        
    # Get the map boundaries
    min_north = map_obj.min_north
    min_east = map_obj.min_east
    max_north = map_obj.max_north
    max_east = map_obj.max_east
        
    # Get the obstacle margin due to all assets ship length
    margin = ship_length/2
            
    # min_bound and max_bound
    n_route_bound = [min_north + margin , max_north - margin]
    e_route_bound = [min_east + margin, max_east - margin]
            
    # Check if position is outside bound
    outside_n = n_pos < n_route_bound[0] or n_pos > n_route_bound[1]
    outside_e = e_pos < e_route_bound[0] or e_pos > e_route_bound[1]
        
    is_outside = outside_n or outside_e
        
    return is_outside
    
def is_grounding(map_obj : PolygonObstacle,
                 pos, 
                 ship_length):
    ''' 
        Checks if the positions is inside any obstacle
    '''
    # Unpack ship position
    n_pos, e_pos = pos
        
    # Get the obstacle margin due to all assets ship length
    # Margin is defined as a square patch enveloping the ships
    margin = ship_length/2
        
    # Get the max reach and min reach of the ship
    min_north = n_pos - margin
    min_east = e_pos - margin
    max_north = n_pos + margin
    max_east = e_pos + margin
        
    # All patch's hard point
    hard_points = [(min_north, min_east), (min_north, max_east), (max_north, min_east), (max_north, max_east)]
        
    is_inside = False
        
    for hard_point in hard_points:
        if map_obj.if_pos_inside_obstacles(hard_point[0], hard_point[1]):
            is_inside =  True
    
    return is_inside
    
def is_point_outside_horizon(map_obj : PolygonObstacle,
                             point):
    ''' 
        Checks if the sampled route are outside the map horizon. 
        Map horizons are determined by the edge point of the determined route point. 
        Allowed additional margin are default 100 m for North and East boundaries.
        Only works with start to end route points method (Two initial points).
    '''
    # Unpack ship position
    n_route, e_route = point
        
    # Get the map boundaries
    min_north = map_obj.min_north
    min_east = map_obj.min_east
    max_north = map_obj.max_north
    max_east = map_obj.max_east
            
    # min_bound and max_bound
    n_route_bound = [min_north , max_north]
    e_route_bound = [min_east, max_east]
        
    # Check if position is outside bound
    outside_n = n_route < n_route_bound[0] or n_route > n_route_bound[1]
    outside_e = e_route < e_route_bound[0] or e_route > e_route_bound[1]
        
    is_outside = outside_n or outside_e
        
    return is_outside
    
def is_point_inside_obstacles(map_obj : PolygonObstacle,
                              point):
    ''' 
        Checks if the tagged position is inside any obstacle
    '''
    is_inside = False
        
    if map_obj.if_pos_inside_obstacles(point[0], point[1]):
        is_inside =  True
      
    return is_inside

def is_ship_navigation_warning(e_ct, e_tol=500):
    '''
        Check if the cross-track error grow bigger than the allowed limit
    '''
    ## Ship deviates off the course beyond tolerance
    is_warning_nav = np.abs(e_ct) > e_tol
        
    return is_warning_nav
    
def is_collision_imminent(test_pos, obs_pos, safety_distance=3000):
    '''
        Check if the incoming obstacle ship is too close to the ship under test
    '''
    n_test, e_test = test_pos
    n_obs, e_obs = obs_pos
    dist_squared = (n_test - n_obs)**2 + (e_test - e_obs)**2
    
    is_near_collision = dist_squared < safety_distance**2 
    
    return is_near_collision
    
def is_ship_collision(own_pos, test_pos, minimum_ship_distance=50):
    ''' 
        If test ship and obstacle ship distance is below some threshold, categorize it as collision
    '''
    # Unpack ship position
    n_test, e_test = own_pos
    n_obs, e_obs = test_pos
    
    # Compute the ship distance
    ship_distance =  (n_test - n_obs)**2 + (e_test - e_obs)**2
        
    # Collision logic
    is_collision = False
        
    if ship_distance < minimum_ship_distance ** 2:
        is_collision = True
        
    return is_collision

def is_sample_travel_dist_too_far(travel_dist, traj_segment_length, distance_factor=2):
    '''
        If the obstacle ship travel distance is bigger than traj_segment_length * distance_factor
        before it even manage to arrive at te next Radius of Accpetance, assume as navigational failure
        
        - traj_segment_length is defined as the length of start-to-end straight trajectory of an obstacle
          ship divided by the maximal intermediate waypoint sampling frequency.
    '''
    is_too_far = travel_dist > traj_segment_length * distance_factor
    return is_too_far

def is_sample_travel_time_too_long(travel_time, time_limit=np.inf):
    '''
        If the obstacle ship failed to reach the next Radius of Acceptance before the time limit, assume
        as navigational failure. However, due to the difficult time limit assumption time limit is defaulted
        as a positive infinite integer
    '''
    is_too_long = travel_time > time_limit
    return is_too_long

def is_reach_radius_of_acceptance(auto_pilot, 
                                  pos, 
                                  r_o_a=200):
    '''
        Check if the ship reach the radius of acceptance region
    '''
    # Unpack the argument
    n_pos, e_pos = pos
        
    ## Obstacle ship's navigation class under autopilot controller class 
    # This belong to dataclass ShipAssets() in env.py
    # use obs.autopilot.navigate
    # - obs.autopilot.navigate.next_wpt for the latest waypoint index
    # - obs.autopilot.navigate.north / obs.autopilot.navigate.east for the waypoint lists
    obs_nav = auto_pilot
    
    # Get the latest waypoint update
    next_wpt = obs_nav.next_wpt
        
    # Compute the distance to the next route
    dist_to_next_route = (n_pos - obs_nav.navigate.north[next_wpt])**2 + \
                         (e_pos - obs_nav.navigate.east[next_wpt])**2
        
    is_reach_roa =  dist_to_next_route < r_o_a**2
    return is_reach_roa 

def is_power_overload(power, available_power):
    '''
    Check if the power overloading happens
    '''
    
    is_power_overload = power > available_power
    return is_power_overload

def is_within_simu_time_limit(asset):
    '''
    Check if the simulation time is within the allowed simulation time limit
    '''
    is_within_time_limit = asset.ship_model.int.time > asset.ship_model.int.sim_time
    
    return is_within_time_limit
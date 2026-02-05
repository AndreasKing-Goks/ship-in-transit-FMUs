"""
Setpoints Manager Python FMU implementation.
This FMU manages a list of setpoints (waypoints and desired speeds) and provides the previous and next setpoints.
It includes a switching mechanism based on a radius of acceptance.
This FMU contains a minimum 2 setpoints to a maximum 10 setpoints.

Authors : Andreas R.G. Sitorus
Date    : Januari 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np

class SetPointsManager(Fmi2Slave):
    
    author = "Andreas R.G. Sitorus"
    description = "Setpoints Manager Python FMU implementation"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        ## Parameters
        self.ra                 = 300.0     # Radius of Acceptance
        self.max_inter_wp       = 8         # Maximum Intermediate Waypoints
        
        ## Waypoints [In Longitude (east) and Latitude (north)]
        # Start
        self.wp_start_north     = np.nan
        self.wp_start_east      = np.nan
        self.wp_start_speed     = np.nan
        
        # Intermediate
        self.wp_1_north         = np.nan
        self.wp_1_east          = np.nan
        self.wp_1_speed         = np.nan
        
        self.wp_2_north         = np.nan
        self.wp_2_east          = np.nan
        self.wp_2_speed         = np.nan
        
        self.wp_3_north         = np.nan
        self.wp_3_east          = np.nan
        self.wp_3_speed         = np.nan
        
        self.wp_4_north         = np.nan
        self.wp_4_east          = np.nan
        self.wp_4_speed         = np.nan
        
        self.wp_5_north         = np.nan
        self.wp_5_east          = np.nan
        self.wp_5_speed         = np.nan
        
        self.wp_6_north         = np.nan
        self.wp_6_east          = np.nan
        self.wp_6_speed         = np.nan
        
        self.wp_7_north         = np.nan
        self.wp_7_east          = np.nan
        self.wp_7_speed         = np.nan
        
        self.wp_8_north         = np.nan
        self.wp_8_east          = np.nan
        self.wp_8_speed         = np.nan
        
        # End
        self.wp_end_north       = np.nan
        self.wp_end_east        = np.nan
        self.wp_end_speed       = np.nan
        
        ## Input Variables
        self.north              = 0.0
        self.east               = 0.0
        
        ## Output Variables
        self.prev_wp_north      = np.nan
        self.prev_wp_east       = np.nan
        self.prev_wp_speed      = np.nan
        
        self.next_wp_north      = np.nan
        self.next_wp_east       = np.nan
        self.next_wp_speed      = np.nan
        
        self.last_wp_active     = False     # Boolean flag if ship reaches the last waypoint.
        
        ## Registration 
        # Parameters
        self.register_variable(Real("ra", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        self.register_variable(Integer("max_inter_wp", causality=Fmi2Causality.parameter,variability=Fmi2Variability.tunable))
        
        # Input variables
        self.register_variable(Real("north", causality=Fmi2Causality.input))
        self.register_variable(Real("east", causality=Fmi2Causality.input))
        
        # Waypoints
        self.register_variable(Real("wp_start_north", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_start_east", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_start_speed", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        
        self.register_variable(Real("wp_1_north", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_1_east", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_1_speed", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        
        self.register_variable(Real("wp_2_north", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_2_east", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_2_speed", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        
        self.register_variable(Real("wp_3_north", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_3_east", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_3_speed", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        
        self.register_variable(Real("wp_4_north", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_4_east", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_4_speed", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        
        self.register_variable(Real("wp_5_north", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_5_east", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_5_speed", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        
        self.register_variable(Real("wp_6_north", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_6_east", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_6_speed", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        
        self.register_variable(Real("wp_7_north", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_7_east", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_7_speed", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        
        self.register_variable(Real("wp_8_north", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_8_east", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_8_speed", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        
        self.register_variable(Real("wp_end_north", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_end_east", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        self.register_variable(Real("wp_end_speed", causality=Fmi2Causality.parameter,variability=Fmi2Variability.fixed))
        
        # Output variables
        self.register_variable(Real("prev_wp_north", causality=Fmi2Causality.output))
        self.register_variable(Real("prev_wp_east", causality=Fmi2Causality.output))
        self.register_variable(Real("prev_wp_speed", causality=Fmi2Causality.output))
        
        self.register_variable(Real("next_wp_north", causality=Fmi2Causality.output))
        self.register_variable(Real("next_wp_east", causality=Fmi2Causality.output))
        self.register_variable(Real("next_wp_speed", causality=Fmi2Causality.output))
        
        self.register_variable(Boolean("last_wp_active", causality=Fmi2Causality.output))
            
            
    def get_trajectory_array(self):
        # Trajectory container
        self.trajectory = [[self.wp_start_north, self.wp_start_east, self.wp_start_speed], [self.wp_end_north, self.wp_end_east, self.wp_end_speed]]
        
        for i in range(self.max_inter_wp):
            north = getattr(self, f"wp_{i+1}_north")
            east  = getattr(self, f"wp_{i+1}_east")
            speed = getattr(self, f"wp_{i+1}_speed")
            
            if (north is not np.nan) and (east is not np.nan) and (speed is not np.nan):
                self.trajectory.insert(-1,[north, east, speed])     # Insert the waypoints in the last entry before the end waypoint
       
                
    def do_step(self, current_time: float, step_size: float) -> bool:
        # Get trajectory array
        self.get_trajectory_array()
        
        for i in range(len(self.trajectory)-1):
            dist_to_wp = np.array([self.trajectory[i][0] - self.north, 
                                   self.trajectory[i][1] - self.east])
            
            if np.linalg.norm(dist_to_wp) <= self.ra:
                self.prev_wp_north = self.trajectory[i][0]
                self.prev_wp_east  = self.trajectory[i][1]
                self.prev_wp_speed = self.trajectory[i][2]
                
                self.next_wp_north = self.trajectory[i+1][0]
                self.next_wp_east  = self.trajectory[i+1][1]
                self.next_wp_speed = self.trajectory[i+1][2]
                
            if (self.next_wp_north == self.trajectory[-1][0]) and (self.next_wp_east == self.trajectory[-1][1]):
                self.last_wp_active = True
            
        return True
                
                
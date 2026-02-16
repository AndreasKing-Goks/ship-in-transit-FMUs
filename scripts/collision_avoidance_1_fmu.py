"""
Collision Avoidance Type 1 Python FMU implementation.
This FMU manages the collision avoidance algorithm.

Authors : Andreas R.G. Sitorus
Date    : February 2026
"""

from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Real, Integer, Boolean, String
import numpy as np

class SimpleCollisionAvoidance(Fmi2Slave):
    
    author = "Andreas R.G. Sitorus"
    description = "Simple Collision Avoidance Algorithm Python FMU Implementation"
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
         
         # Parameters
        self.throttle_scale_factor  = 0.0
        self.rud_ang_increment_deg  = 0.0
        self.danger_zone_radius     = 0.0
        self.collision_zone_radius  = 0.0
        
        # Input
        self.own_north              = 0.0
        self.own_east               = 0.0
        self.own_yaw_angle          = 0.0
        self.own_measured_speed     = 0.0
        
""" 
This module provides utilities class for the simulator visualization
"""

import numpy as np

class ShipDraw:
    ''' This class is used to calculate the coordinates of each
        corner of 80 meter long and 20 meter wide ship seen from above,
        and rotate and translate the coordinates according to
        the ship heading and position
    '''

    def __init__(self, l, b):
        self.l = l
        self.b = b

    def local_coords(self, scale = 1.0):
        ''' Here the ship is pointing along the local
            x-axis with its center of origin (midship)
            at the origin
            1 denotes the left back corner
            2 denotes the left starting point of bow curvatiure
            3 denotes the bow
            4 the right starting point of the bow curve
            5 the right back cornier
        '''
        x1, y1 = -self.l / 2, -self.b / 2
        x2, y2 = self.l / 4, -self.b / 2
        x3, y3 = self.l / 2, 0.0
        x4, y4 = self.l / 4, self.b / 2
        x5, y5 = -self.l / 2, self.b / 2

        x = np.array([x1, x2, x3, x4, x5, x1]) * scale
        y = np.array([y1, y2, y3, y4, y5, y1]) * scale
        return x, y

    def rotate_coords(self, x, y, psi):
        ''' Rotates the ship an angle psi
        '''
        x_t = np.cos(psi) * x - np.sin(psi) * y
        y_t = np.sin(psi) * x + np.cos(psi) * y
        return x_t, y_t

    def translate_coords(self, x_ned, y_ned, north, east):
        ''' Takes in coordinates of the corners of the ship (in the ned-frame)
            and translates them in the north and east direction according to
            "north" and "east"
        '''
        x_t = x_ned + north
        y_t = y_ned + east
        return x_t, y_t
    
def ship_snap_shot(north, east, yaw_angle):
    draw = ShipDraw()
    x, y = draw.local_coords()
    x_ned, y_ned = draw.rotate_coords(x, y, yaw_angle)
    x_ned_trans, y_ned_trans = draw.translate_coords(x_ned, y_ned, north, east)
    
    return x_ned_trans, y_ned_trans
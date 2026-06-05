'''
The scripts contains classes of reward function designs from :

https://www.sciencedirect.com/science/article/pii/S2405896323012855

By Goto, T., et al
'''

import numpy as np

class RewardDesign1():
    def __init__(self, target, offset_param):
        self.target = target
        self.offset_param = offset_param
        
    def __call__(self, val):
        reward = np.exp(-(val - self.target)**2 /  self.offset_param)
        return reward
        
class RewardDesign2():
    def __init__(self, target, offset_param1, offset_param2):
        self.target = target
        self.offset_param1 = offset_param1
        self.offset_param2 = offset_param2
        
    def __call__(self, val):
        if val < self.target:
            reward = np.exp(-(val - self.target)**2 /  self.offset_param1)
        else:
            reward = np.exp(-(val - self.target)**2 /  self.offset_param2)
        return reward
    
class RewardDesign3():
    def __init__(self, target, offset_param):
        self.target = target
        self.offset_param = offset_param
        
    def __call__(self, val):
        if val < self.target:
            reward = np.exp(-(val - self.target)**2 /  self.offset_param)
        else:
            reward = 1
        return reward
    
class RewardDesign4():
    def __init__(self, target, offset_param):
        self.target = target
        self.offset_param = offset_param
        
    def __call__(self, val):
        if val < self.target:
            reward = 1
        else:
            reward = np.exp(-(val - self.target)**2 /  self.offset_param)
        return reward
    
class RewardDesign5():
    def __init__(self, target_bound_low, target_bound_high, offset_param):
        self.target_bound_low = target_bound_low
        self.target_bound_high = target_bound_high
        self.offset_param = offset_param
        
    def __call__(self, val):
        if val < self.target_bound_low:
            reward = np.exp(-(val - self.target_bound_low)**2 /  self.offset_param)
        elif self.target_bound_low <= val < self.target_bound_high:
            reward = 1
        elif val >= self.target_bound_high:    
            reward = np.exp(-(val - self.target_bound_high)**2 /  self.offset_param)
        return reward
    
class RewardDesign6():
    def __init__(self, target1, target2, second_peak, flat_zone, offset_param1, offset_param2, offset_param3, offset_param4):
        self.target1 = target1
        self.target2 = target2
        self.second_peak = second_peak
        self.flat_zone = flat_zone
        self.offset_param1 = offset_param1
        self.offset_param2 = offset_param2
        self.offset_param3 = offset_param3
        self.offset_param4 = offset_param4
        
    def __call__(self, val):
        if val < self.target1:
            reward = np.exp(-(val - self.target1)**2 /  self.offset_param1)
        elif self.target1 <= val < ((self.target1 + self.target2)/2):
            reward = (1 - self.flat_zone) * np.exp(-(val - self.target1)**2 /  self.offset_param2) + self.flat_zone
        elif ((self.target1 + self.target2)/2) <= val < self.target2:
            reward = (self.second_peak - self.flat_zone) * np.exp(-(val - self.target2)**2 /  self.offset_param3) + self.flat_zone
        elif val >= self.target2:
            reward = self.second_peak * np.exp(-(val - self.target2)**2 /  self.offset_param4)
        
        return reward
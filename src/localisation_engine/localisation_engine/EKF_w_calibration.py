from abc import ABC, abstractmethod
import numpy as np
from custom_msgs.msg import Distances
from builtin_interfaces.msg import Time


from EKF_proto import EKF_proto_c

class EKF_w_callibration(EKF_proto_c):
    
    
    def __init__(self, mode : str, anchors : np.ndarray,  q_std = 0.1, r_std=0.1):
            
        pass
        
    
    def get_new_state_sync_(self, msg : Distances):
        pass

    
    def get_new_state_async_(self, msg : Distances):
        pass
    
    
    def predict_sync_(self, dt : float):
        pass
    
    
    def predict_async_(self, dt : float):
        pass
    
    
    def update_sync_(self, distances  :np.ndarray):
        pass
    
    
    def update_async_(self, distances  :np.ndarray):
        pass
    
    
    def get_new_state(self, distances : Distances):
        pass
    
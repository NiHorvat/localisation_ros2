from abc import ABC, abstractmethod
import numpy as np
from custom_msgs.msg import Distances


class EKF_proto_c:
    
    
    
    @abstractmethod
    def __init__(self, mode : str, anchors : np.ndarray, robot_start_position, q_std = 0.1, r_std=0.1):
        pass
    
    @abstractmethod
    def get_new_state_sync_(self, msg : Distances):
        pass

    @abstractmethod
    def get_new_state_async_(self, msg : Distances):
        pass
    
    @abstractmethod
    def predict_sync_(self, dt : float):
        pass
    
    @abstractmethod
    def predict_async_(self, dt : float):
        pass
    
    @abstractmethod
    def update_sync_(self, distances  :np.ndarray):
        pass
    
    @abstractmethod
    def update_async_(self, distances  :np.ndarray):
        pass
    
    @abstractmethod
    def get_new_state(self, distances : Distances):
        pass
    
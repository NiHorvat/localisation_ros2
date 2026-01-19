import numpy as np
from custom_msgs.msg import Distances
from builtin_interfaces.msg import Time

class EKF_c:
    """
    class containign all the logic for Extended Kalman Filter calculation
    """

    def __init__(self, anchors : np.ndarray, q_std = 0.1, r_std=0.1):
        self.anchors_ = anchors                             # anchor coords
        self.n_ = anchors.shape[0]                          # number of anchors
        self.x_ = np.zeros((6, 1), dtype=np.float32)        # state vector

        self.Q_ = np.eye(6) * (q_std**2)                    # process noise covariance matrix
        self.R_ = np.eye(self.n_) * (r_std**2)              # sensor noise covariance matrix

        self.P_ = np.eye(6) * 1.0                           # prediction covariance
        
        self.last_timestamp_ = Time()
        self.last_timestamp_.sec = 0
        self.last_timestamp_.nanosec = 0


        self.last_timestamp_.sec = 0
        self.last_timestamp_.nanosec = 0
        
        
    def predict(self, dt : float) -> np.ndarray:
        
        F = np.eye(N=6,dtype=np.float32) # transition matrix
        F[0, 3] = F[1, 4] = F[2, 5] = dt

        # apriori x prediction

        # x = F * x
        self.x_ = np.dot(F, self.x_)
        
        # P = F * P * F^T + Q
        self.P_ = np.dot(np.dot(F, self.P_), F.T) + self.Q_




    def update(self, measurements : np.ndarray) -> np.ndarray:
        
        # measurements is a list/array of distances to each anchor
        z = np.array(measurements).reshape(-1, 1)
        
        
        # h(x): Predicted distances based on current state
        hx = np.zeros((self.n_, 1))
        # H: Jacobian matrix
        H = np.zeros((self.n_, 6))

        for i in range(self.n_):
            anchor = self.anchors_[i]
            dx = self.x_[0, 0] - anchor[0]
            dy = self.x_[1, 0] - anchor[1]
            dz = self.x_[2, 0] - anchor[2]
            
            dist = np.sqrt(dx**2 + dy**2 + dz**2)
            
            # Avoid division by zero
            if dist < 1e-6: dist = 1e-6
            
            hx[i, 0] = dist
            
            # Partial derivatives for position
            H[i, 0] = dx / dist
            H[i, 1] = dy / dist
            H[i, 2] = dz / dist
            # Derivatives for velocity are 0
            H[i, 3] = H[i, 4] = H[i, 5] = 0

        # EKF Update Equations
        e = z - hx  # Innovation (Error)
        S = np.dot(np.dot(H, self.P_), H.T) + self.R_ # Innovation Covariance
        K = np.dot(np.dot(self.P_, H.T), np.linalg.inv(S)) # Kalman Gain
        
        self.x_ = self.x_ + np.dot(K, e)
        self.P_ = np.dot((np.eye(6) - np.dot(K, H)), self.P_)
        



    def get_new_state(self, measurements : Distances) -> np.array:    
        
        dt = Time()
        dt.sec = measurements.stamp.sec - self.last_timestamp_.sec
        dt.nanosec = measurements.stamp.nanosec - self.last_timestamp_.nanosec
        dt_float = dt.sec + dt.nanosec * 1e-9

        self.predict(dt=dt_float)
        self.update(measurements=measurements.distances)
        
        # update the value for last_timestamp_
        self.last_timestamp_ = measurements.stamp


        return self.x_
    

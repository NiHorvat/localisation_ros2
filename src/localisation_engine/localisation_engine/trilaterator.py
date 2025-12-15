import numpy as np

class Trilaterator():


    def __init__(self,anchor_coords : np.ndarray):
        
        self.anchor_coords = anchor_coords

    
    
    def trilaterate(self, distances : np.ndarray):

        p1,p2,p3 = self.anchor_coords
        r1,r2,r3 = distances
        def norm(v):
            return np.sqrt(np.sum(v**2))
        def dot(v1,v2):
            return np.dot(v1,v2)
        def cross(v1,v2):
            return np.cross(v1,v2)

        ex = (p2-p1) / norm(p2-p1)
        i = dot(ex, p3-p1)
        a = (p3-p1) - ex*i
        ey = a / norm(a)
        ez = cross(ex, ey)
        d = norm(p2-p1)
        j = dot(ey, p3-p1)
        x = (r1**2 - r2**2 + d**2) / (2*d)
        y = (r1**2 - r3**2 + i**2 + j**2) / (2*j) - (i/j) * x
        b = r1**2 - x**2 - y**2


        if (np.abs(b) < 0.0000000001):
            b = 0

        z = np.sqrt(b)
        if np.isnan(z):
            return None

        a = p1 + ex*x + ey*y
        p4a = a + ez*z
        p4b = a - ez*z

        return p4a, p4b



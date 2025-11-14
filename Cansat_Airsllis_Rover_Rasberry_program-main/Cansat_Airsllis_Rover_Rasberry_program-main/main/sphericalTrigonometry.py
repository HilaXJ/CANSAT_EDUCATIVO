
import math
import numpy as np

def degreesToRadians(degrees):
    return degrees * math.pi / 180

class SphericalPoint():
    def __init__(self, latitude = 0 , longitude = 0):
            self.latitude = latitude
            self.longitude = longitude

    def bearingTo(self, destination):
        phi1 = degreesToRadians(self.latitude)
        phi2 = degreesToRadians(destination.latitude)
        delta_lambda = degreesToRadians(destination.longitude - self.longitude)

        x = np.cos( phi1 ) * np.sin( phi2 ) - np.sin( phi1 ) * np.cos( phi2 ) * np.cos( delta_lambda )
        y = np.sin( delta_lambda ) * np.cos( phi2 )

        theta = math.atan2(y, x)

        return theta
    
    def toECEF(self, radius = 6378137):
        phi = degreesToRadians(self.latitude)
        _lambda = degreesToRadians(self.longitude)

        x = radius * np.cos(phi) * np.cos(_lambda)
        y = radius * np.cos(phi) * np.sin(_lambda)
        z = radius * np.sin(phi)

        return np.array([x, y, z])

    def toENU(self, referencePoint, radius = 6378137):
        phi_r = degreesToRadians(referencePoint.latitude)
        lambda_r = degreesToRadians(referencePoint.longitude)

        T = np.array([ 
            [-np.sin(lambda_r), np.cos(lambda_r), 0],
            [-np.sin(phi_r) * np.cos(lambda_r), -np.sin(phi_r) * np.sin(lambda_r), np.cos(phi_r)],
            [ np.cos(phi_r) * np.cos(lambda_r),  np.cos(phi_r) * np.sin(lambda_r), np.sin(phi_r)]
            ])

        return T @ (self.toECEF(radius) - referencePoint.toECEF(radius))


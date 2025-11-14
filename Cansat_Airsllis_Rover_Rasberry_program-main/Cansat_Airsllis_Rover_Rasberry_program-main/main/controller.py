import math
from sphericalTrigonometry import SphericalPoint
import os
class PIDController():
    def __init__(self, robot):
        self.robot = robot
        self.speed = 0.5
        self.kp = 5
        self.ki = 0.1
        self.kd = 0
        self.previous_error = 0
        self.integral_error = 0
        self.debug = True  # activar/desactivar prints


    def control(self, target):
        # Usar la última posición GPS disponible en memoria (no bloqueante)
        current_point = getattr(self.robot.gps, 'last_point', None)
        if current_point is None or (getattr(current_point, 'latitude', 0.0) == 0.0 and getattr(current_point, 'longitude', 0.0) == 0.0):
            # Si no hay datos válidos, mantener el control anterior (o puedes retornar 0,0)
            print("⚠ No hay datos GPS válidos. Manteniendo velocidad y giro anteriores.")
            return self.speed, 0.0

        target_theta = current_point.bearingTo(target)
        target_theta = math.atan2(math.sin(target_theta), math.cos(target_theta))
        current_heading = self.robot.bno055.get_heading_radians()
        current_heading = math.atan2(math.sin(current_heading), math.cos(current_heading))
        u_theta = target_theta - current_heading
        current_error = math.atan2(math.sin(u_theta), math.cos(u_theta))
        differential_error = current_error - self.previous_error
        integral_error = current_error + self.integral_error

        self.previous_error = current_error
        self.integral_error = integral_error

        w = current_error * self.kp + differential_error * self.kd + integral_error * self.ki
        if self.debug:
            try:
                print(
                    f"PID | lat={current_point.latitude:.6f} lon={current_point.longitude:.6f} "
                    f"target_bearing={math.degrees(target_theta):.3f} heading={math.degrees(current_heading):.3f} "
                    f"err={current_error:.3f} derr={differential_error:.3f} ierr={self.integral_error:.3f} "
                    f"speed={self.speed:.3f} w={w:.3f}"
                )
            except Exception:
                pass

        return self.speed, w

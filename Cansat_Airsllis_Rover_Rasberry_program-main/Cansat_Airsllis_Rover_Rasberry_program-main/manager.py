import math
import time
import numpy as np
from filter import KalmanFilter

class RoverManager():
    def __init__(self, robot, controller, target):
        self.robot = robot
        self.controller = controller
        self.previous_left_ticks = 0
        self.previous_right_ticks = 0
        self.target = target
        self.filter = KalmanFilter(self.robot)

    def distance_per_ticks(self, ticks, wheel_radius, ticks_per_revolution):
        return 2 * math.pi * wheel_radius / ticks_per_revolution * ticks

    def update_odometry(self, dt = 0.01):

        left_encoder_ticks = self.robot.left_encoder.ticks()
        right_encoder_ticks = self.robot.right_encoder.ticks()

        #print("left encoder: ", left_encoder_ticks)
        #print("right encoder: ", right_encoder_ticks)


        left_wheel_distance = self.distance_per_ticks(left_encoder_ticks - self.previous_left_ticks, self.robot.wheel_radius, self.robot.left_encoder.tpr)
        right_wheel_distance = self.distance_per_ticks(right_encoder_ticks - self.previous_right_ticks, self.robot.wheel_radius, self.robot.right_encoder.tpr)


        distance = (left_wheel_distance + right_wheel_distance ) / 2
        phi = (right_wheel_distance - left_wheel_distance) / self.robot.wheel_base_length


        dx = distance * math.cos(self.robot.theta)
        dy = distance * math.sin(self.robot.theta)
        dtheta = phi


        self.robot.x += dx
        self.robot.y += dy
        self.robot.theta += dtheta
        self.robot.theta = math.atan2(math.sin(self.robot.theta), math.cos(self.robot.theta))
        self.robot.speed = abs(distance) / dt


        self.previous_left_ticks = left_encoder_ticks
        self.previous_right_ticks = right_encoder_ticks


    def unicycle_to_differential(self, v, w):
        left_speed = (2 * v + w * self.robot.wheel_base_length) / (2 * self.robot.wheel_radius)
        right_speed = (2 * v - w * self.robot.wheel_base_length) / (2 * self.robot.wheel_radius)
        return left_speed, right_speed

    

    def ensure_wheel_speeds(self, left_speed, right_speed, w):
        if( max(left_speed, right_speed) > self.robot.max_speed ):
            if left_speed > right_speed:
                left_speed = self.robot.max_speed
                right_speed = w * self.robot.wheel_base_length / self.robot.wheel_radius  + left_speed
            else:
                right_speed = self.robot.max_speed
                left_speed = right_speed - w * self.robot.wheel_base_length / self.robot.wheel_radius

        elif( min(left_speed, right_speed) < -self.robot.max_speed ):
            if left_speed < right_speed:
                left_speed = -self.robot.max_speed
                right_speed = w * self.robot.wheel_base_length / self.robot.wheel_radius  + left_speed
            else:
                right_speed = -self.robot.max_speed
                left_speed = right_speed - w * self.robot.wheel_base_length / self.robot.wheel_radius

        return left_speed, right_speed



    def execute_with_filter(self, dt = 0.01, gps_enabled = False):
        v, w = self.controller.control(self.target)

        max_w = 2 * self.robot.wheel_radius * min(self.robot.max_left_wheel_speed, self.robot.max_right_wheel_speed) / self.robot.wheel_base_length
        w = max(min(w, max_w), -max_w)

        #print("\n" + "="*36)
        print(f"[CONTROL] Max wheel speed allowed: {self.robot.max_left_wheel_speed}")
        print(f"[CONTROL] Linear speed (v):        {v:.3f} m/s")
        print(f"[CONTROL] Max angular control (w): {max_w:.3f} rad/s")
        print(f"[CONTROL] Angular control (w):     {w:.3f} rad/s")

        left_speed, right_speed = self.unicycle_to_differential(v, w)

        print(f"[DIFF]   Raw wheel speeds:   L={left_speed:.3f} rad/s, R={right_speed:.3f} rad/s")
        max_wheel = max(abs(left_speed), abs(right_speed))
        if max_wheel > self.robot.max_speed:
            scale = self.robot.max_speed / max_wheel
            left_speed *= scale
            right_speed *= scale
        #left_speed, right_speed = self.ensure_wheel_speeds(left_speed, right_speed, w)

        print(f"[DIFF]   Scaled wheel speeds: L={left_speed:.3f} rad/s, R={right_speed:.3f} rad/s")
        #print("="*36 + "\n")

        self.robot.update_speed(left_speed, right_speed)
        
        #time.sleep(dt)

        #self.update_odometry()
        """
        print("predict state: ", self.robot.get_state())

        self.filter.predict(self.robot.get_state())

        current_compass = self.robot.bno055.get_heading_radians()
        current_compass = math.atan2( math.sin(current_compass), math.cos(current_compass) )

        measurements = np.array([
            [current_compass]
        ])

        if gps_enabled:
            spherical_point, _ = self.robot.gps.read()  
            current_gps = spherical_point.toENU(self.robot.reference) 
            measurements = np.array([
                [current_gps[0], current_gps[1], current_compass]
            ])

        print("measurements: ", measurements)

        state = self.filter.update(self.robot.get_state(), measurements, only_compass = (not gps_enabled) )
        self.robot.set_state(state)

        print("update state: ", state)
        """


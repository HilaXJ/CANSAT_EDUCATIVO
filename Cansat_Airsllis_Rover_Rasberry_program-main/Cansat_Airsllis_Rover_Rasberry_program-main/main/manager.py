import math
import time
import numpy as np

class RoverManager():
    def __init__(self, robot, controller, target):
        self.robot = robot
        self.controller = controller
        self.previous_left_ticks = 0
        self.previous_right_ticks = 0
        self.target = target

    def unicycle_to_differential(self, v, w):
        left_speed = (2 * v + w * self.robot.wheel_base_length) / (2 * self.robot.wheel_radius)
        right_speed = (2 * v - w * self.robot.wheel_base_length) / (2 * self.robot.wheel_radius)
        return left_speed, right_speed


    def execute_control_gps(self):
        v, w = self.controller.control(self.target)

        max_w = 2 * self.robot.wheel_radius * min(self.robot.max_left_wheel_speed, self.robot.max_right_wheel_speed) / self.robot.wheel_base_length
        w = max(min(w, max_w), -max_w)

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

        print(f"[DIFF]   Scaled wheel speeds: L={left_speed:.3f} rad/s, R={right_speed:.3f} rad/s")
        self.robot.update_speed(left_speed, right_speed)



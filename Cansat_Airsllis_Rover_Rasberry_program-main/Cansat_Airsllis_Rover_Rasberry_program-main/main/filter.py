import numpy as np
import math

class KalmanFilter():
    def __init__(self, robot):
        self.robot = robot

        self.H = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [0, 0, 1, 0, 0]
        ])

        self.H_compass = np.array([
            [0, 0, 1, 0, 0]
        ])

        self.P = np.array([
            [1000, 0, 0, 0, 0],
            [0, 1000, 0, 0, 0],
            [0, 0, 100, 0, 0],
            [0, 0, 0, 1000, 0],
            [0, 0, 0, 0, 1000]
        ])

        self.Q = np.identity(5)

        self.R = np.array([
            [100, 0, 0],
            [0, 100, 0],
            [0, 0, 25]
        ])

        self.R_compass = np.array([
            [25]
        ])

    def predict(self, x, dt = 0.01):

        speed = self.robot.speed
        theta = self.robot.theta

        F = np.array([
            [1, 0, -speed * dt * np.sin(theta), dt * np.cos(theta), 0],
            [0, 1,  speed * dt * np.cos(theta), dt * np.sin(theta), 0],
            [0, 0,                           1,                  0, dt],
            [0, 0,                           0,                  1, 0],
            [0, 0,                           0,                  0, 1]
        ])
        #x = f(x, u)
        self.P = F @ self.P @ F.transpose() + self.Q

        return x

    def update(self, x, z, only_compass = False):

        H = self.H
        R = self.R

        if only_compass:
            H = self.H_compass
            R = self.R_compass

            if x[2, 0] < -math.pi / 2 and z[0, 0] > math.pi / 2:
                x[2, 0] += 2 * math.pi
            elif x[2, 0] > math.pi / 2 and z[0, 0] < math.pi / 2:
                 z[0, 0] += 2 * math.pi

        I = np.identity(5)

        y = z - H @ x
        S = H @ self.P @ H.transpose() + R
        K = self.P @ H.transpose() @ np.linalg.inv(S)

        x = x + K @ y
        self.P = (I - K @ H) @ self.P

        return x


    def filter(self, x, z):

        x = self.update(z)

        x = self.predict(x)

        return x




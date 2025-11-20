import numpy as np
import time
import math

class EKF_NoDrift:
    def __init__(self, start_x, start_y, start_theta):
        # Estado: [x (Este), y (Norte), theta (Heading)]
        self.x = np.array([[start_x], 
                           [start_y], 
                           [start_theta]])

        # Incertidumbre inicial
        self.P = np.eye(3) * 1.0

        # --- TUNING (Ajuste fino) ---
        # Q: ¿Cuánto confiamos en la odometría (encoders)?
        # Valores altos = confiamos poco (el filtro buscará más al GPS)
        self.Q = np.diag([0.05, 0.05, 0.02]) 

        # R: ¿Cuánto confiamos en el GPS?
        # Valor estándar ~2.0 metros de error
        self.R_gps = np.diag([2.0, 2.0])
        
        self.last_time = time.time()

    def predict(self, v, w):
        """
        Predicción usando Odometría (Encoders).
        Sistema: Heading (0=Norte, CW+)
        """
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        if dt > 1.0: dt = 0.01 

        theta = self.x[2, 0]

        # Modelo cinemático para Heading (0=N, X=E, Y=N)
        # dx = v * sin(theta)
        # dy = v * cos(theta)
        # dtheta = w
        
        self.x[0, 0] += v * np.sin(theta) * dt
        self.x[1, 0] += v * np.cos(theta) * dt
        self.x[2, 0] += w * dt

        # Normalizar ángulo (-pi a pi)
        self.x[2, 0] = math.atan2(math.sin(self.x[2, 0]), math.cos(self.x[2, 0]))

        # Jacobiano F (Cómo cambia el estado)
        F = np.array([[1, 0,  v * np.cos(theta) * dt],
                      [0, 1, -v * np.sin(theta) * dt],
                      [0, 0, 1]])

        # Predicción de covarianza
        self.P = F @ self.P @ F.T + self.Q

    def update_gps(self, gps_x, gps_y):
        """ Corrección usando GPS """
        z = np.array([[gps_x], [gps_y]])
        H = np.array([[1, 0, 0],
                      [0, 1, 0]]) # Solo medimos X, Y

        y = z - (H @ self.x) # Error de predicción
        S = H @ self.P @ H.T + self.R_gps
        K = self.P @ H.T @ np.linalg.inv(S) # Ganancia Kalman

        self.x = self.x + (K @ y)
        self.P = (np.eye(3) - (K @ H)) @ self.P

    def get_state(self):
        return self.x[0, 0], self.x[1, 0], self.x[2, 0]



import math
from sphericalTrigonometry import SphericalPoint
import os
import time
class PIDController():
    def __init__(self, robot):
        self.robot = robot
        self.speed = 0.5
        self.kp = 5
        self.ki = 0.001
        self.kd = 0
        self.previous_error = 0
        self.integral_error = 0
        self.debug = True  # activar/desactivar prints


    def control(self, target):
        # Usar la ltima posicin GPS disponible en memoria (no bloqueante)
        current_point = getattr(self.robot.gps, 'last_point', None)
        if current_point is None or (getattr(current_point, 'latitude', 0.0) == 0.0 and getattr(current_point, 'longitude', 0.0) == 0.0):
            # Si no hay datos vlidos, mantener el control anterior (o puedes retornar 0,0)
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
    

class PIDController_1(): 
    def __init__(self, robot):
        self.robot = robot
        
        # --- CONFIGURACIÓN ---
        # Kp un poco más alto para que gire con fuerza sin tener que frenar
        self.kp = 4.0   
        self.ki = 0.0   # Cero para evitar memoria del pasado (giro loco)
        self.kd = 0.1   # Pequeño freno al acercarse al ángulo correcto
        
        # Velocidad constante deseada
        self.base_speed = 0.6  # Ajusta esto según tu preferencia (m/s)
        
        # Variables internas
        self.previous_error = 0
        self.integral_error = 0
        self.last_time = time.time()
        self.max_integral = 5.0 
        self.debug = True 
#KALMAN FILTRE
# AGREGAR ESTO DENTRO DE TU CLASE PIDController (ej. PIDController_1)
    
    def control_xy(self, target_x, target_y):
        """
        Control PID usando coordenadas locales (Metros).
        Calcula el Heading deseado usando atan2(dx, dy).
        """
        # 1. Tiempo dt
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        if dt <= 0: dt = 0.001

        # 2. Estado Actual (Viene del filtro a través de robot.x/y)
        current_x = self.robot.x
        current_y = self.robot.y
        current_heading = self.robot.bno055.get_heading_radians() # O self.robot.theta filtrado

        # 3. Calcular error de posición (Metros)
        dx = target_x - current_x
        dy = target_y - current_y
        
        # 4. Calcular Heading Deseado
        # En tu sistema (0=N, CW+), el ángulo es atan2(Este, Norte) -> atan2(dx, dy)
        target_heading = math.atan2(dx, dy)
        
        # 5. Error de Ángulo
        u_theta = target_heading - current_heading
        current_error = math.atan2(math.sin(u_theta), math.cos(u_theta))

        # 6. PID
        differential_error = (current_error - self.previous_error) / dt
        
        if abs(current_error) < 0.5:
            self.integral_error += current_error * dt
        else:
            self.integral_error = 0.0
        self.integral_error = max(min(self.integral_error, 5.0), -5.0)

        w = (current_error * self.kp) + (self.integral_error * self.ki) + (differential_error * self.kd)
        
        self.previous_error = current_error
        
        # Velocidad constante (ajusta base_speed en tu __init__)
        v = self.base_speed 

        return v, w
    def control(self, target):
        # 1. Tiempo real (dt)
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        if dt <= 0: dt = 0.001

        # 2. Obtener GPS
        current_point = getattr(self.robot.gps, 'last_point', None)
        if current_point is None or (getattr(current_point, 'latitude', 0.0) == 0.0):
            # Si pierde GPS, sigue avanzando recto (v=base) pero sin giro (w=0)
            return self.base_speed, 0.0

        # 3. Calcular Ángulos
        target_theta = current_point.bearingTo(target)
        target_theta = math.atan2(math.sin(target_theta), math.cos(target_theta))
        
        current_heading = self.robot.bno055.get_heading_radians()
        current_heading = math.atan2(math.sin(current_heading), math.cos(current_heading))
        
        # Diferencia de ángulo
        u_theta = target_theta - current_heading
        current_error = math.atan2(math.sin(u_theta), math.cos(u_theta))

        # 4. PID (Estabilizado)
        differential_error = (current_error - self.previous_error) / dt
        
        # Anti-Windup simple
        if abs(current_error) < 0.5: 
            self.integral_error += current_error * dt
        else:
            self.integral_error = 0.0 # Reset si el error es grande
            
        self.integral_error = max(min(self.integral_error, self.max_integral), -self.max_integral)

        # Salida de Giro (w)
        w = (current_error * self.kp) + (self.integral_error * self.ki) + (differential_error * self.kd)
        
        self.previous_error = current_error

        # 5. VELOCIDAD CONSTANTE (Lo que pediste)
        # El robot SIEMPRE avanza. 
        # Nota: Si el error es > 90 grados, el robot avanzará "alejándose" mientras gira 
        # para hacer una curva en U, como un avión.
        v = self.base_speed

        if self.debug:
            try:
                print(
                    f"PID | Err: {math.degrees(current_error):.1f}° "
                    f"| Out -> V:{v:.2f} W:{w:.2f} (Siempre avanzando)"
                )
            except Exception:
                pass

        return v, w
    



class PIDController_2():
    def __init__(self, robot):
        self.robot = robot
        
        # --- CONFIGURACIÓN DE GANANCIAS ---
        self.kp = 4.0   # Potencia de giro inmediata
        self.ki = 0.0   # ¡IMPORTANTE! Empieza en 0.0 para máxima estabilidad
        self.kd = 0.1   # Ayuda a frenar el giro cuando se acerca al objetivo
        
        # Velocidad base
        self.base_speed = 0.5
        
        # Variables de estado PID
        self.previous_error = 0
        self.integral_error = 0
        self.last_time = time.time()
        
        # Límites de seguridad
        self.max_integral = 5.0  # Techo máximo para la memoria (evita giro loco)
        self.debug = True 

    def control(self, target):
        # 1. Calcular el tiempo transcurrido real (dt)
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        if dt <= 0: dt = 0.001 # Evitar división por cero

        # 2. Obtener datos GPS
        current_point = getattr(self.robot.gps, 'last_point', None)
        
        # Verificación de seguridad GPS
        if current_point is None or (getattr(current_point, 'latitude', 0.0) == 0.0 and getattr(current_point, 'longitude', 0.0) == 0.0):
            print("⚠ [PID] Sin GPS válido. Deteniendo robot por seguridad.")
            return 0.0, 0.0 # ¡Mejor parar que seguir a ciegas!

        # 3. Cálculo de Ángulos (Matemática de Navegación)
        target_theta = current_point.bearingTo(target)
        target_theta = math.atan2(math.sin(target_theta), math.cos(target_theta))
        
        current_heading = self.robot.bno055.get_heading_radians()
        current_heading = math.atan2(math.sin(current_heading), math.cos(current_heading))
        
        # Cálculo del error (Normalizado de -pi a pi)
        u_theta = target_theta - current_heading
        current_error = math.atan2(math.sin(u_theta), math.cos(u_theta))

        # 4. Lógica PID Estabilizada
        
        # Derivada: Cambio de error por unidad de tiempo
        differential_error = (current_error - self.previous_error) / dt
        
        # Integral: Acumulación controlada (Anti-Windup)
        # Solo acumulamos si el error es pequeño (para ajuste fino).
        # Si el error es grande, reiniciamos la integral para no confundir al robot.
        if abs(current_error) < 0.5: # Solo integra si el error es < 30 grados
            self.integral_error += current_error * dt
        else:
            self.integral_error = 0.0
            
        # Limitar (Clamp) la integral para que nunca sea gigante
        self.integral_error = max(min(self.integral_error, self.max_integral), -self.max_integral)

        # Cálculo de la salida Angular (w)
        w = (current_error * self.kp) + (self.integral_error * self.ki) + (differential_error * self.kd)
        
        self.previous_error = current_error

        # 5. Control de Velocidad Inteligente (Smart Speed)
        # Si el error de ángulo es grande (> 45 grados aprox 0.8 rad), 
        # la velocidad lineal (v) baja a 0 para girar en el sitio.
        if abs(current_error) > 0.8:
            v = 0
        else:
            # Acelera a medida que el error disminuye
            # Si error es 0 -> v = base_speed
            # Si error es 0.8 -> v = 0
            factor = 1.0 - (abs(current_error) / 0.8)
            v = self.base_speed * max(0.0, factor)

        if self.debug:
            try:
                print(
                    f"PID | Err: {math.degrees(current_error):.1f}° "
                    f"| P:{current_error*self.kp:.2f} I:{self.integral_error*self.ki:.2f} D:{differential_error*self.kd:.2f} "
                    f"| Out -> V:{v:.2f} W:{w:.2f}"
                )
            except Exception:
                pass

        return v, w

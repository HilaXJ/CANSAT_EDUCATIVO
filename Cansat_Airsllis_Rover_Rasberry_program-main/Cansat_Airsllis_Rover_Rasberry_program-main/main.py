
import time
import subprocess
from gpiozero import Motor
import numpy as np
from robot import Robot
from encoder import QuadratureEncoder
from gps import GPS
from bno055 import BNO055
from bme_280 import BME280Sensor
from ina226_sensor import INA226Sensor
from controller import PIDController
from manager import RoverManager
from sphericalTrigonometry import SphericalPoint
from calibration import Calibration
from gpiozero import OutputDevice
import os
def flatten_dict(d, parent_key='', sep='_'):
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep))
        else:
            items.append((new_key, v))
    return items

def truncate_value(key, value):
    # Latitud / longitud => 5 decimales
    if isinstance(value, float):
        if any(x in key for x in ["latitude", "longitude"]):
            return round(value, 5)
        # Cualquier otro float => 1 decimal
        return round(value, 1)
    # Tuplas => cada elemento a 1 decimal si es float
    if isinstance(value, tuple):
        return tuple(round(v, 1) if isinstance(v, float) else v for v in value)
    return value

DATA_FILE = "data_to_send.txt"
def main():
    dt = 0.01   # 100 Hz (puedes ajustar según tu loop)
    min_altitude_air = 10  # meters
    # Reference altitude variables
    alt_ref_bme = None
    alt_ref_gps = None
    # === Motores ===
    LEFT_MOTOR_INPUT = (18, 12)
    RIGHT_MOTOR_INPUT = (13, 19)
    left_motor = Motor(LEFT_MOTOR_INPUT[0], LEFT_MOTOR_INPUT[1])
    right_motor = Motor(RIGHT_MOTOR_INPUT[0], RIGHT_MOTOR_INPUT[1])

    # === Nicrom GPIO ===
    nicrom = OutputDevice(17, active_high=True, initial_value=False)

    # === LEDs indicadores ===
    led1 = OutputDevice(23, active_high=True, initial_value=False)  # LED indicador 1
    led2 = OutputDevice(24, active_high=True, initial_value=False)  # LED indicador 2

    # === Encoders ===
    LEFT_ENCODER_INPUT = {'hall_sensor_A':27, 'hall_sensor_B': 22, 'ticks_per_revolution': 985}
    RIGHT_ENCODER_INPUT = {'hall_sensor_A': 5, 'hall_sensor_B': 6, 'ticks_per_revolution': 985}
    left_encoder = QuadratureEncoder(
        LEFT_ENCODER_INPUT['hall_sensor_A'],
        LEFT_ENCODER_INPUT['hall_sensor_B'],
        LEFT_ENCODER_INPUT['ticks_per_revolution']
    )
    right_encoder = QuadratureEncoder(
        RIGHT_ENCODER_INPUT['hall_sensor_A'],
        RIGHT_ENCODER_INPUT['hall_sensor_B'],
        RIGHT_ENCODER_INPUT['ticks_per_revolution']
    )


    gps = GPS()
    bno055 = BNO055()
    bme280 = BME280Sensor()
    ina226 = INA226Sensor()

    robot = Robot(left_motor, right_motor, left_encoder, right_encoder, gps, bno055, bme280, ina226)
    controller = PIDController(robot)

 
    target = SphericalPoint(40.87894444, -119.12336111)
    rover_manager = RoverManager(robot, controller, target)
    calibration=Calibration(robot)
    tasks=["sensorCalibration",
           "inAir",
           "nicrom",
           "GPSControl",
           "CamaraControl"]
    
    currently_task=tasks[0]
    epoch = 0
    secondary_started = False
    secondary_proc = None
    try:
        # Para detección de aterrizaje por baja aceleración
        low_accel_start_time = None
        # === Reference altitude measurement ===
        N_REF = 10
        gps_altitudes = []
        print("Measuring reference GPS altitude (10 samples)...")
        while len(gps_altitudes) < N_REF:
            alt = robot.gps.last_alt
            if alt is not None and alt != 0.0:
                gps_altitudes.append(alt)
                print(f"GPS sample {len(gps_altitudes)}/{N_REF}: {alt:.2f} m")
            else:
                print("Esperando fix GPS...")
            time.sleep(0.5)  # 500 ms entre muestras
        alt_ref_gps = np.mean(gps_altitudes)
        print(f"Reference GPS altitude (mean of {len(gps_altitudes)}): {alt_ref_gps:.2f} m")
        
        
        while True:
            if currently_task == "sensorCalibration":
                # leds on
                led2.on()
                sensors_data = calibration.get_values()
                print("\n=== SENSOR CALIBRATION DATA ===")
                for key, value in sensors_data.items():
                    if isinstance(value, dict):
                        print(f"{key.upper()}:")
                        for subkey, subvalue in value.items():
                            print(f"   {subkey}: {subvalue}")
                    else:
                        print(f"{key}: {value}")
                print("===============================\n")

                # Check for launch: GPS altitude >100m above reference
                cond_gps = False
                gps_current = robot.gps.last_alt
                gps_diff = None
                if gps_current is not None and gps_current != 0.0:
                    gps_diff = gps_current - alt_ref_gps
                    print(f"[Launch check] GPS altitude diff: {gps_diff:.2f} m (current: {gps_current:.2f}, ref: {alt_ref_gps:.2f})")
                    if abs(gps_diff) > 100:
                        cond_gps = True
                if cond_gps:
                    print("Launch detected → Switching to inAir")
                    currently_task = "inAir"
                    
                    if not secondary_started:
                        try:
                            secondary_proc = subprocess.Popen(["python3", "lora_emisor.py"])  # ejecutar en paralelo
                            secondary_started = True
                            print("[INFO] Proceso secundario 'lora_emisor.py' iniciado en paralelo.")
                        except Exception as e:
                            print(f"[ERROR] No se pudo iniciar lora_emisor.py: {e}")    
            elif currently_task == "inAir":
                led2.off()
                sensors_data = calibration.get_values()
                print("\n=== SENSOR CALIBRATION DATA ===")
                for key, value in sensors_data.items():
                    if isinstance(value, dict):
                        print(f"{key.upper()}:")
                        for subkey, subvalue in value.items():
                            print(f"   {subkey}: {subvalue}")
                    else:
                        print(f"{key}: {value}")
                print("===============================\n")
                # Check for landing: only low linear acceleration (BNO055)
                cond_accel = False
                epsilon = 0.2
                lin_accel = None
                if "bno055" in sensors_data and "linear_acceleration" in sensors_data["bno055"]:
                    lin_accel = sensors_data["bno055"]["linear_acceleration"]
                    if (
                        lin_accel is not None and
                        isinstance(lin_accel, (list, tuple)) and
                        all(a is not None for a in lin_accel) and
                        all(abs(a) < epsilon for a in lin_accel)
                    ):
                        # Si ya estábamos en baja aceleración, comprobar tiempo
                        if low_accel_start_time is None:
                            low_accel_start_time = time.time()
                        elif time.time() - low_accel_start_time >= 10:
                            print("Landing detected (low acceleration for 30s) → Switching to nicrom")
                            currently_task = "nicrom"
                            low_accel_start_time = None
                    else:
                        # Si la aceleración sube, reiniciar temporizador
                        low_accel_start_time = None

            elif currently_task == "nicrom":
                
                led2.on()
                print("Activating nicrom")
                nicrom.on()
                time.sleep(40)
                nicrom.off()
                led2.off()
                print("Nicrom deactivated. Proceeding to sensor calibration and GPSControl.")
                time.sleep(5)
                # Calibrate sensors (as before)
                """
                sensors_data = calibration.get_values()
                print("\n=== SENSOR CALIBRATION DATA ===")
                for key, value in sensors_data.items():
                    if isinstance(value, dict):
                        print(f"{key.upper()}:")
                        for subkey, subvalue in value.items():
                            print(f"   {subkey}: {subvalue}")
                    else:
                        print(f"{key}: {value}")
                print("===============================\n")
                """
                #time.sleep(10)
                robot.active_calibration_bno055()
                """
                sensors_data = calibration.get_values()
                print("\n=== SENSOR CALIBRATION DATA ===")
                for key, value in sensors_data.items():
                    if isinstance(value, dict):
                        print(f"{key.upper()}:")
                        for subkey, subvalue in value.items():
                            print(f"   {subkey}: {subvalue}")
                    else:
                        print(f"{key}: {value}")
                print("===============================\n")
                """
                currently_task = "GPSControl"

            elif currently_task == "GPSControl":
                #start = time.time()
                print("epoch: %d" % epoch)
                #gps_enabled = (epoch > 0 and (epoch % 100 == 0))
                rover_manager.execute_with_filter(gps_enabled=False)
                epoch += 1
                current_point = robot.gps.last_point 
                distancia = np.linalg.norm(current_point.toENU(target))
                
                if distancia <= 4:
                    print("Objetivo alcanzado (dentro de 5 metros)")
                    robot.stop()
                    currently_task = tasks[0]
                
                #elapsed = time.time() - start
                #time.sleep(max(0, dt - elapsed))
                time.sleep(0.01)

            elif currently_task == "CamaraControl":
                pass
            
            sensors_data = calibration.get_values()
            flat = flatten_dict(sensors_data)
            # Excluir claves que contienen 'encoder' (singular o plural)
            filtered = [(k, v) for k, v in flat if 'encoders' not in k.lower()]
            values = [truncate_value(k, v) for k, v in filtered]
            str_values = [str(v) for v in values]
            csv_payload = ','.join(str_values)
            with open("data_to_send.txt", 'w') as f:
                f.write(csv_payload + '\n')
            time.sleep(dt)
            
            
    except KeyboardInterrupt:
        print("\n🛑 Misión interrumpida por el usuario.")
        robot.update_speed(0, 0)
        if secondary_proc and secondary_proc.poll() is None:
            print("Cerrando proceso secundario...")
            secondary_proc.terminate()
            try:
                secondary_proc.wait(timeout=5)
            except Exception:
                pass



if __name__ == "__main__":
    main()


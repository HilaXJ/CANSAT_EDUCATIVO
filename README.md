# CANSAT_EDUCATIVO – Rover (Raspberry Pi + Coral)

Rover diferencial para misión ARLISS/CanSat con navegación por GPS/IMU, control de motores con encoders, telemetría LoRa y **detección visual de cono final** usando **Raspberry Pi + Picamera2 + Google Coral USB (EdgeTPU)**.

> 📌 Este README resume la estructura, dependencias y ejecución del proyecto. Los detalles finos (parámetros, pines, estrategias de control) se explican dentro del código y comentarios.

---

## ✨ Características principales

- **FSM / Estados de misión**
  - `sensorCalibration` – lectura/calibración rápida de sensores.
  - `inAir` – detección de vuelo/caída y aterrizaje por baja aceleración (BNO055).
  - `nicrom` – activación de nicrom para liberación.
  - `GPSControl` – navegación por GPS/IMU hasta objetivo.
  - `CamaraControl` – activa **on-demand** cámara + Coral a ≤ 5 m para detección de cono y **control fino**.

- **Sensores**
  - **GPS** (lat/lon/alt; distancia a objetivo).
  - **BNO055** (orientación + aceleración lineal).
  - **BME280** (temperatura, presión, altitud).
  - **INA226** (corriente/voltaje).
  - **Encoders** en ambas ruedas (odometría).

- **Actuadores**
  - Motores DC con PWM (diferencial).
  - Salida **nicrom** (GPIO).
  - LEDs de estado.

- **Visión (activable)**
  - **Picamera2** + **PyCoral** en EdgeTPU.
  - Modelo TFLite: `ssd_mobilenet_v1_cone700_edgetpu.tflite` + `labels700.txt`.

- **Telemetría**
  - Archivo `data_to_send.txt` (CSV plano).
  - Proceso paralelo `lora_emisor.py` (emisión LoRa).

---

## 🧰 Hardware (referencia)

- Raspberry Pi 4B / Zero 2 W  
- Google Coral USB Accelerator  
- Cámara CSI (v2/v3, Picamera2)  
- BNO055 (I²C), BME280 (I²C), INA226 (I²C)  
- GPS (UART0: GPIO14/15)  
- LoRa (p. ej., en UART3)  
- Drivers de motor + encoders  
- Nicrom + driver/MOSFET adecuado  
- Batería / regulación según diseño

> Verifica masa común (GND), protección a picos en motores y dimensionamiento de cables.

---

## 📦 Software / Dependencias

- Raspberry Pi OS (Bullseye/Bookworm)
- Python 3.9+
- Paquetes (ejemplo):
  - `gpiozero`, `numpy`, `opencv-python`, `picamera2`
  - `smbus2`, `pyserial` (si aplica)
  - **PyCoral** (`pycoral`) + `tflite_runtime` compatible con EdgeTPU

Instalación típica (ajústala a tu entorno):

```bash
sudo apt update
sudo apt install -y python3-pip python3-opencv python3-numpy python3-picamera2 \
                    python3-smbus i2c-tools git
# EdgeTPU runtime + PyCoral: seguir guía oficial según tu OS/CPU
pip3 install gpiozero smbus2 pyserial
pip3 install pycoral

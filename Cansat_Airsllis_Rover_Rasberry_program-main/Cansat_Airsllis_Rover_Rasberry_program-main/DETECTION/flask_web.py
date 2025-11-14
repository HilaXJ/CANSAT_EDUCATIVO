from flask import Flask, Response
import cv2, time

DEV="/dev/video4"; W,H,FPS=1280,720,30

cap = cv2.VideoCapture(DEV, cv2.CAP_V4L2)
for fourcc in ("MJPG","YUYV"):
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    if cap.isOpened(): print(f"OK {DEV} {W}x{H}@{FPS} {fourcc}"); break
else:
    raise SystemExit("No pude abrir la cámara.")

app = Flask(__name__)

@app.route("/")
def index(): return '<img src="/video_feed">'

@app.route("/video_feed")
def feed():
    def gen():
        while True:
            ok, frame = cap.read()
            if not ok: time.sleep(0.01); continue
            ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if not ok: continue
            yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpg.tobytes() + b"\r\n")
    return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001, threaded=True)

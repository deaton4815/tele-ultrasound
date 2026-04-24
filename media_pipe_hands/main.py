import cv2
import mediapipe as mp
import time
import socket
import json
import threading

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)
MATLAB_IP = '127.0.0.1'
MATLAB_PORT = 5010

BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
HandLandmarkerResult = mp.tasks.vision.HandLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode

latest_result = None
result_lock = threading.Lock()

def result_callback(result: HandLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
    global latest_result
    with result_lock:
        latest_result = result

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path='hand_landmarker.task'),
    running_mode=VisionRunningMode.LIVE_STREAM,
    num_hands=1,
    result_callback=result_callback
)

count = -1
last_timestamp_ms = 0

with HandLandmarker.create_from_options(options) as landmarker:
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        count += 1
        ret, frame = cap.read()
        if not ret:
            break

        timestamp_ms = max(int(time.time() * 1000), last_timestamp_ms + 1)
        last_timestamp_ms = timestamp_ms

        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        landmarker.detect_async(mp_image, timestamp_ms)

        with result_lock:
            current_result = latest_result
            latest_result = None

        if current_result and current_result.hand_landmarks:
            lm_index_mcp = current_result.hand_landmarks[0][5]

            x = 1 - lm_index_mcp.x
            y = lm_index_mcp.y
            if count % 100 == 0:
                print(f"x: {x:.3f}, y: {y:.3f}")

            data = json.dumps([x, y])
            try:
                sock.sendto((data + '\n').encode(), (MATLAB_IP, MATLAB_PORT))
            except BlockingIOError:
                pass

            # ADD POINT ON THE KNUCKLE
            h, w, _ = frame.shape
            px = int((1 - x) * w)
            py = int(y * h)
            cv2.circle(frame, (px, py), 8, (0, 255, 0), -1)

            # ADD COORDINATES TEXT (BIGGER)
            cv2.putText(frame, f"({x:.3f}, {y:.3f})", (px + 10, py - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 3)

        frame_large = cv2.resize(frame, None, fx=1.5, fy=1.5)
        cv2.imshow('Hand Landmarker', frame_large)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
import cv2
import mediapipe as mp
import time
import socket
import json

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)  # Non-blocking so send never freezes the thread
MATLAB_IP = '127.0.0.1'
MATLAB_PORT = 5005

BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
HandLandmarkerResult = mp.tasks.vision.HandLandmarkerResult
VisionRunningMode = mp.tasks.vision.RunningMode

latest_result = None

def result_callback(result: HandLandmarkerResult, output_image: mp.Image, timestamp_ms: int):
    global latest_result
    latest_result = result  # Just store — don't send from callback thread

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path='hand_landmarker.task'),
    running_mode=VisionRunningMode.LIVE_STREAM,
    num_hands=1,
    result_callback=result_callback
)

with HandLandmarker.create_from_options(options) as landmarker:
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        timestamp_ms = int(time.time() * 1000)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)
        landmarker.detect_async(mp_image, timestamp_ms)

        # Consume and clear latest_result in the main loop
        if latest_result and latest_result.hand_landmarks:
            landmarks = []
            for lm in latest_result.hand_landmarks[0]:
                landmarks.append([lm.x, lm.y, lm.z])
                print(f"x: {lm.x:.3f}, y: {lm.y:.3f}, z: {lm.z:.3f}")

            data = json.dumps(landmarks)
            try:
                sock.sendto((data + '\n').encode(), (MATLAB_IP, MATLAB_PORT))
            except BlockingIOError:
                pass  # Drop the packet if the buffer is momentarily full — better than blocking

            latest_result = None  # Clear after sending

        cv2.imshow('Hand Landmarker', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
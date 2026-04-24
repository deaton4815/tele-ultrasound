import cv2
import mediapipe as mp
import time
import socket
import json
import threading


MATLAB_IP = "127.0.0.1"
MATLAB_PORT = 5010

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)

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


def open_camera():
    for idx in [0, 1, 2, 3]:
        print(f"Trying camera {idx}...")
        cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)

        if cap.isOpened():
            ret, frame = cap.read()
            if ret and frame is not None:
                print(f"Camera {idx} opened.")
                return cap

        cap.release()

    return None


def main():
    global latest_result

    options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path="hand_landmarker.task"),
        running_mode=VisionRunningMode.LIVE_STREAM,
        num_hands=1,
        result_callback=result_callback,
    )

    cap = open_camera()

    if cap is None:
        print("ERROR: Could not open webcam.")
        input("Press Enter to exit...")
        return

    count = -1
    last_timestamp_ms = 0

    try:
        with HandLandmarker.create_from_options(options) as landmarker:
            print("Python hand tracker running.")
            print("Press q or ESC in the camera window to quit.")
            print("You can also press CTRL+C in PowerShell.")

            while True:
                count += 1

                ret, frame = cap.read()
                if not ret or frame is None:
                    print("ERROR: Camera frame not received.")
                    break

                timestamp_ms = max(int(time.time() * 1000), last_timestamp_ms + 1)
                last_timestamp_ms = timestamp_ms

                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                mp_image = mp.Image(
                    image_format=mp.ImageFormat.SRGB,
                    data=rgb_frame,
                )

                landmarker.detect_async(mp_image, timestamp_ms)

                with result_lock:
                    current_result = latest_result
                    latest_result = None

                if current_result and current_result.hand_landmarks:
                    lm_index_mcp = current_result.hand_landmarks[0][5]

                    x = 1 - lm_index_mcp.x
                    y = lm_index_mcp.y

                    data = json.dumps([x, y])

                    try:
                        sock.sendto((data + "\n").encode(), (MATLAB_IP, MATLAB_PORT))
                    except BlockingIOError:
                        pass

                    h, w, _ = frame.shape
                    px = int(lm_index_mcp.x * w)
                    py = int(lm_index_mcp.y * h)

                    cv2.circle(frame, (px, py), 10, (0, 255, 0), -1)
                    cv2.putText(
                        frame,
                        f"x={x:.2f}, y={y:.2f}",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1,
                        (0, 255, 0),
                        2,
                    )

                    if count % 30 == 0:
                        print(f"x: {x:.3f}, y: {y:.3f}")

                cv2.imshow("Hand Landmarker", frame)

                # Quit if user closes the OpenCV window
                if cv2.getWindowProperty("Hand Landmarker", cv2.WND_PROP_VISIBLE) < 1:
                    print("Camera window closed.")
                    break

                key = cv2.waitKey(1) & 0xFF

                if key == ord("q") or key == 27:
                    print("Quit key pressed.")
                    break

    except KeyboardInterrupt:
        print("\nCTRL+C detected. Exiting...")

    finally:
        print("Cleaning up Python camera app...")
        cap.release()
        cv2.destroyAllWindows()
        sock.close()
        print("Python hand tracker stopped.")


if __name__ == "__main__":
    main()
import cv2
import urllib.request
import numpy as np
import time

# Configuration
URL = 'http://192.168.185.83'
# CAM_URL = URL + '/cam-low.jpg'
# CAM_URL = URL + '/cam-mid.jpg'
CAM_URL = URL + '/cam-hi.jpg'
LED_ON_URL = URL + '/api/control?pos=led_on'
LED_OFF_URL = URL + '/api/control?pos=led_off'
WINDOW_NAME = "ESP32-CAM Stream"
RECONNECT_DELAY = 1  # seconds
MAX_RETRIES = 5

# Flashlight state
flashlight_on = False

def toggle_flashlight():
    global flashlight_on
    url = LED_ON_URL if not flashlight_on else LED_OFF_URL
    try:
        with urllib.request.urlopen(url, timeout=2) as response:
            result = response.read().decode()
            print(f"Flashlight {'ON' if not flashlight_on else 'OFF'}: {result}")
            flashlight_on = not flashlight_on
    except Exception as e:
        print(f"Failed to toggle flashlight: {e}")

def main():
    global flashlight_on
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    retry_count = 0
    last_frame_time = time.time()
    
    while True:
        try:
            # Fetch the image
            with urllib.request.urlopen(CAM_URL, timeout=2) as img_resp:
                img_np = np.array(bytearray(img_resp.read()), dtype=np.uint8)
                frame = cv2.imdecode(img_np, -1)
                
                if frame is None:
                    print("Empty frame received")
                    continue
                    
                # Calculate FPS
                current_time = time.time()
                fps = 1 / (current_time - last_frame_time)
                last_frame_time = current_time
                
                # Display FPS on frame
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Display Flashlight status
                status = "ON" if flashlight_on else "OFF"
                cv2.putText(frame, f"Flashlight: {status}", (10, 65),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
                
                # Show the frame
                cv2.imshow(WINDOW_NAME, frame)
                retry_count = 0  # Reset retry counter on success
                
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('f'):
                    toggle_flashlight()
                    
        except Exception as e:
            print(f"Error: {e}")
            retry_count += 1
            if retry_count >= MAX_RETRIES:
                print("Max retries reached, exiting...")
                break
            print(f"Retrying in {RECONNECT_DELAY} seconds...")
            time.sleep(RECONNECT_DELAY)
            
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

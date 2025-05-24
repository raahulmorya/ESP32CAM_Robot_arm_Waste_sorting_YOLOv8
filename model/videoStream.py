import cv2
import urllib.request
import numpy as np
import time

# Configuration
URL = 'http://192.168.185.83'
# CAM_URL = URL + '/cam-low.jpg'
# CAM_URL = URL + '/cam-mid.jpg'
CAM_URL = URL + '/cam-hi.jpg'
WINDOW_NAME = "ESP32-CAM Stream"
RECONNECT_DELAY = 1  # seconds
MAX_RETRIES = 5

def main():
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
                
                # Show the frame
                cv2.imshow(WINDOW_NAME, frame)
                retry_count = 0  # Reset retry counter on success
                
                # Exit on 'q' key
                if cv2.waitKey(1) == ord('q'):
                    break
                    
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
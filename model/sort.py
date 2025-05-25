import cv2
import urllib.request
import numpy as np
import time
from ultralytics import YOLO

# Configuration
URL = 'http://192.168.185.83'
CAM_URL = URL + '/cam-hi.jpg'
SORT_URL = URL + '/api/sort'
STATUS_URL = URL + '/api/status'
LED_ON_URL = URL + '/api/control?pos=led_on'
LED_OFF_URL = URL + '/api/control?pos=led_off'
WINDOW_NAME = "ESP32-CAM Stream"
RECONNECT_DELAY = 1
MAX_RETRIES = 5

# Model classes
CLASS_NAMES = {
    0: 'battery', 1: 'can', 2: 'cardboard_bowl', 3: 'cardboard_box',
    4: 'chemical_plastic_bottle', 5: 'chemical_plastic_gallon', 6: 'chemical_spray_can',
    7: 'light_bulb', 8: 'paint_bucket', 9: 'plastic_bag', 10: 'plastic_bottle',
    11: 'plastic_bottle_cap', 12: 'plastic_box', 13: 'plastic_cultery', 14: 'plastic_cup',
    15: 'plastic_cup_lid', 16: 'reuseable_paper', 17: 'scrap_paper', 18: 'scrap_plastic',
    19: 'snack_bag', 20: 'stick', 21: 'straw'
}

RECYCLABLE = ['cardboard_box', 'can', 'plastic_bottle_cap', 'plastic_bottle', 'reuseable_paper']
NON_RECYCLABLE = ['plastic_bag', 'scrap_paper', 'stick', 'plastic_cup', 'snack_bag', 'plastic_box',
                 'straw', 'plastic_cup_lid', 'scrap_plastic', 'cardboard_bowl', 'plastic_cultery']
HAZARDOUS = ['battery', 'chemical_spray_can', 'chemical_plastic_bottle', 'chemical_plastic_gallon',
             'light_bulb', 'paint_bucket']

# Flashlight state
flashlight_on = False

# Load YOLO model
model = YOLO('model.pt')  # Replace with your model path

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

def check_arm_status():
    try:
        with urllib.request.urlopen(STATUS_URL, timeout=2) as response:
            result = response.read().decode()
            return 'busy":false' in result.lower()
    except:
        return False

def send_sort_request(item_type):
    try:
        with urllib.request.urlopen(f"{SORT_URL}?type={item_type}", timeout=2) as response:
            return response.read().decode()
    except Exception as e:
        print(f"Failed to send sort request: {e}")
        return None

def main():
    global flashlight_on
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    retry_count = 0
    last_frame_time = time.time()
    last_detection_time = 0
    detection_cooldown = 5  # seconds
    
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

                # Display Flashlight status
                status = "ON" if flashlight_on else "OFF"
                cv2.putText(frame, f"Flashlight: {status}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
                # Perform detection only if arm is ready and cooldown has passed
                if check_arm_status() and (current_time - last_detection_time) > detection_cooldown:
                    # Run YOLO inference
                    results = model(frame)
                    
                    for result in results:
                        boxes = result.boxes.xyxy
                        classes = result.boxes.cls
                        confidences = result.boxes.conf
                        
                        for box, cls, conf in zip(boxes, classes, confidences):
                            class_idx = int(cls.item())
                            confidence = conf.item()
                            class_name = CLASS_NAMES.get(class_idx, 'unknown')
                            
                            # Determine category
                            if class_name in HAZARDOUS:
                                category = "HAZARDOUS"
                                color = (0, 0, 255)  # Red
                                if confidence > 0.7:  # Only act on high confidence
                                    print(f"Detected {class_name}, sending hazardous sort request")
                                    send_sort_request("hazardous")
                                    last_detection_time = time.time()
                                    break
                            elif class_name in NON_RECYCLABLE:
                                category = "NON_RECYCLABLE"
                                color = (0, 165, 255)  # Orange
                                if confidence > 0.3:
                                    print(f"Detected {class_name}, sending non-recyclable sort request")
                                    send_sort_request("nonrecyclable")
                                    last_detection_time = time.time()
                                    break
                            else:
                                category = "RECYCLABLE"
                                color = (0, 255, 0)  # Green
                            
                            # Draw bounding box and label
                            x1, y1, x2, y2 = map(int, box)
                            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                            label = f"{class_name} {confidence:.2f} - {category}"
                            cv2.putText(frame, label, (x1, y1 - 10), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                
                # Display FPS on frame
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Show the frame
                cv2.imshow(WINDOW_NAME, frame)
                retry_count = 0
                
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
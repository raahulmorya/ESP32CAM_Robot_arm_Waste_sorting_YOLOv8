import cv2
import urllib.request
import numpy as np
import time
from ultralytics import YOLO

# Configuration
URL = 'http://192.168.185.83'
CAM_URL = URL + '/cam-hi.jpg'
LED_ON_URL = URL + '/api/control?pos=led_on'
LED_OFF_URL = URL + '/api/control?pos=led_off'
WINDOW_NAME = "ESP32-CAM Stream"
RECONNECT_DELAY = 1  # seconds
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

def classify_object(frame):
    try:
        # Run YOLO inference
        results = model(frame)
        
        # Process results
        for result in results:
            boxes = result.boxes.xyxy  # Bounding boxes
            classes = result.boxes.cls  # Class indices
            confidences = result.boxes.conf  # Confidence scores
            
            for box, cls, conf in zip(boxes, classes, confidences):
                class_idx = int(cls.item())
                confidence = conf.item()
                class_name = CLASS_NAMES.get(class_idx, 'unknown')
                
                # Determine category
                if class_name in RECYCLABLE:
                    category = 'RECYCLABLE'
                elif class_name in NON_RECYCLABLE:
                    category = 'NON_RECYCLABLE'
                elif class_name in HAZARDOUS:
                    category = 'HAZARDOUS'
                else:
                    category = 'UNKNOWN'
                
                # Draw bounding box and label
                x1, y1, x2, y2 = map(int, box)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{class_name} ({confidence:.2f}) - {category}"
                cv2.putText(frame, label, (x1, y1 - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return frame  # Return the annotated frame

    except Exception as e:
        print(f"Error during classification: {e}")
        return frame  # Return original frame if error occurs

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
                
                # Perform classification and get annotated frame
                frame = classify_object(frame)
                
                # Display FPS on frame
                cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                # Display Flashlight status
                status = "ON" if flashlight_on else "OFF"
                cv2.putText(frame, f"Flashlight: {status}", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                
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
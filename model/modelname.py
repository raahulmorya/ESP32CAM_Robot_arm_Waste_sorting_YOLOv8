from ultralytics import YOLO
model = YOLO("model.pt")
print(model.names)  # Displays the class names the model was trained on
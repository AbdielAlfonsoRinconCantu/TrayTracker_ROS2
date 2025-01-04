from ultralytics import YOLO

model_path = input("Enter the YOLO model file name (e.g., 'yolov8n.pt'): ")

try:
    model = YOLO(model_path)

    # Get the class names
    class_names = model.names

    # Print class names and the total number of classes
    print("Class Names:", class_names)
    print("Number of Classes:", len(class_names))

except Exception as e:
    print(f"An error occurred: {e}")

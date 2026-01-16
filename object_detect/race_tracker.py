import numpy as np
np.bool = bool # Fix for TensorRT/Numpy error
import pyzed.sl as sl
import cv2
from ultralytics import YOLO

# 1. Load Model
model = YOLO("./best.engine", task="detect")

# 2. Setup ZED
zed = sl.Camera()
init_params = sl.InitParameters()
init_params.depth_mode = sl.DEPTH_MODE.ULTRA 
init_params.coordinate_units = sl.UNIT.METER
zed.open(init_params)

zed.enable_positional_tracking(sl.PositionalTrackingParameters())
obj_param = sl.ObjectDetectionParameters()
obj_param.detection_model = sl.OBJECT_DETECTION_MODEL.CUSTOM_BOX_OBJECTS
obj_param.enable_tracking = True
zed.enable_object_detection(obj_param)

# Objects for loop
image_zed = sl.Mat()
objects = sl.Objects()
runtime_params = sl.RuntimeParameters()

print("Visualizer Active. Press 'q' to quit.")

while True:
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        # Retrieve image
        zed.retrieve_image(image_zed, sl.VIEW.LEFT)
        frame = image_zed.get_data()[:, :, :3] # BGR frame for OpenCV
        render_frame = frame.copy()

        # YOLO Inference
        results = model.predict(source=frame, conf=0.4, imgsz=640, verbose=False)

        # Ingest to ZED
        zed_boxes = []
        for r in results:
            for box in r.boxes:
                b = box.xyxy[0].cpu().numpy().astype(int)
                # Draw YOLO 2D Box (Green)
                cv2.rectangle(render_frame, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 2)
                
                tmp = sl.CustomBoxObjectData()
                tmp.bounding_box_2d = np.array([[b[0], b[1]], [b[2], b[1]], [b[2], b[3]], [b[0], b[3]]])
                tmp.label = int(box.cls[0])
                tmp.probability = float(box.conf[0])
                zed_boxes.append(tmp)

        zed.ingest_custom_box_objects(zed_boxes)
        zed.retrieve_objects(objects)

        # Draw ZED 3D Data
        for obj in objects.object_list:
            if obj.tracking_state == sl.OBJECT_TRACKING_STATE.OK:
                # Get 2D position for text overlay
                pos_2d = obj.bounding_box_2d[0] 
                dist = obj.position[2]
                vel_z = obj.velocity[2]
                
                # Overlay Text: ID, Distance, and Velocity
                label = f"ID:{obj.id} Dist:{dist:.1f}m Vz:{vel_z:.1f}m/s"
                cv2.putText(render_frame, label, (int(pos_2d[0]), int(pos_2d[1]-10)), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Display Window
        cv2.imshow("F1TENTH Racing Monitor", render_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

zed.close()
cv2.destroyAllWindows()
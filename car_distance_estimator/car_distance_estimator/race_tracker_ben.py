import numpy as np
np.bool = bool  # Fix for TensorRT/Numpy error

import pyzed.sl as sl
import cv2
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


def choose_raceline_id(cx_px, img_w, deadband=0.15):
    u = cx_px / float(img_w)  # 0..1
    if u < 0.5 - deadband:
        return 1  # links -> innen
    if u > 0.5 + deadband:
        return 3  # rechts -> außen
    return 2      # mitte


# 1) Load Model
model = YOLO("./best.engine", task="detect")

# 2) Setup ZED
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

# parameters
vehicle_height = 0.2       # real height in meters
focal_length = 528.15      # focal length of zed2 (fy)
offset_bb_measure = 0.17   # in meters
offset_stereo_cam = 0.2    # in meters

# raceline decision params
DIST_SWITCH_MAX = 1.3      # only decide left/right if final dist < ...m
DEADBAND = 0.15            # +-...% around image center = "middle"

# ROS2 publisher
rclpy.init()
node = Node("race_tracker_raceline_pub")
pub = node.create_publisher(Int32, "/raceline_id", 10)
msg_id = Int32()

print("Visualizer Active. Press 'q' to quit.")

while True:
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image_zed, sl.VIEW.LEFT)
        frame = image_zed.get_data()[:, :, :3]
        render_frame = frame.copy()

        # YOLO Inference
        results = model.predict(source=frame, conf=0.4, imgsz=640, verbose=False)

        # Ingest to ZED (nur Boxen)
        zed_boxes = []
        for r in results:
            for box in r.boxes:
                b = box.xyxy[0].cpu().numpy().astype(int)

                cv2.rectangle(render_frame, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 2)

                tmp = sl.CustomBoxObjectData()
                tmp.bounding_box_2d = np.array([[b[0], b[1]], [b[2], b[1]], [b[2], b[3]], [b[0], b[3]]])
                tmp.label = int(box.cls[0])
                tmp.probability = float(box.conf[0])
                zed_boxes.append(tmp)

        zed.ingest_custom_box_objects(zed_boxes)
        zed.retrieve_objects(objects)

        # Default: middle
        best_dist = float("inf")
        best_cx = None

        # Draw ZED 3D Data
        for obj in objects.object_list:
            if obj.tracking_state == sl.OBJECT_TRACKING_STATE.OK:
                pos_2d = obj.bounding_box_2d[0]
                dist_stereo = obj.position[2] - offset_stereo_cam
                vel_z = obj.velocity[2]

                # BB-height aus obj.bounding_box_2d
                bb = obj.bounding_box_2d
                xs = [p[0] for p in bb]
                ys = [p[1] for p in bb]
                bbox_height_px = max(ys) - min(ys)

                dist_from_bb = None
                if bbox_height_px > 0:
                    dist_from_bb = vehicle_height * focal_length / bbox_height_px - offset_bb_measure

                # Override-logic
                dist = dist_stereo
                if dist_from_bb is not None and dist_from_bb < 1.0:
                    dist = dist_from_bb

                # Track "best" object (closest) for raceline decision
                if dist < best_dist:
                    best_dist = dist
                    best_cx = 0.5 * (min(xs) + max(xs))

                # Overlay Text
                label = f"ID:{obj.id} Dist:{dist:.2f}m Vz:{vel_z:.1f}m/s"
                cv2.putText(render_frame, label,
                            (int(pos_2d[0]), int(pos_2d[1]-10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                if dist_from_bb is not None:
                    size_label = f"H:{bbox_height_px:.0f}px Dist_bb:{dist_from_bb:.2f}m Dist_used:{dist:.2f}m"
                    cv2.putText(render_frame, size_label,
                                (int(pos_2d[0]), int(pos_2d[1]+15)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Decide raceline_id (only if close enough)
        raceline_id = 2
        if best_cx is not None and best_dist < DIST_SWITCH_MAX:
            print("frame width %d.",frame.shape[1])
            raceline_id = choose_raceline_id(best_cx, frame.shape[1], deadband=DEADBAND)

        # Publish
        msg_id.data = int(raceline_id)
        pub.publish(msg_id)
        rclpy.spin_once(node, timeout_sec=0.0)

        # Debug overlay for chosen raceline
        cv2.putText(render_frame, f"RL:{raceline_id} (1=in,2=mid,3=out)",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow("F1TENTH Racing Monitor", render_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

zed.close()
cv2.destroyAllWindows()
node.destroy_node()
rclpy.shutdown()
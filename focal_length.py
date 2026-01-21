import pyzed.sl as sl

zed = sl.Camera()
init = sl.InitParameters()
init.camera_resolution = sl.RESOLUTION.HD720
zed.open(init)

calib = zed.get_camera_information().camera_configuration.calibration_parameters

fx = calib.left_cam.fx
fy = calib.left_cam.fy
cx = calib.left_cam.cx
cy = calib.left_cam.cy

print(fx, fy, cx, cy)

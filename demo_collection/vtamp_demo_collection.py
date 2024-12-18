import cv2
from pyk4a import PyK4A
import time
import sys
import os

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(current_dir)

from marker_detection import get_kinect_ir_frame, detect_aruco_markers, estimate_transformation, get_kinect_rgb_frame
import numpy as np
import open3d as o3d
import os

DEMO = 10

def get_kinect_rgbd_frame(device, visualize=False):
    """
    Capture an IR frame from the Kinect camera.
    """
    # Capture an IR frame
    rgb_frame = None
    capture = None
    for i in range(20):
        try:
            device.get_capture()
            capture = device.get_capture()
            if capture is not None:
                ir_frame = capture.ir

                # depth_frame = capture.depth
                # ---
                depth_frame = capture.transformed_depth
                # ---
                
                # cv2.imshow('IR', ir_frame)
                rgb_frame = capture.color
                gray_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2GRAY)
                gray_frame = np.clip(gray_frame, 0, 5e3) / 5e3  # Clip and normalize
                # cv2.imshow('color', rgb_frame)
                
                ir_frame_norm = np.clip(ir_frame, 0, 5e3) / 5e3  # Clip and normalize
                pcd_frame = capture.transformed_depth_point_cloud
                # print(pcd_frame.shape, ir_frame.shape)
                # print("successful capture")
                return ir_frame, rgb_frame, ir_frame_norm, pcd_frame, depth_frame
        except:
            time.sleep(0.1)
            # print("Failed to capture IR frame.")
    else:
        # print("Failed to capture IR frame after 20 attempts.")
        return None


k4a = PyK4A(device_id=0)
k4a.start()


def record_rgbd():
    for i in range(50):
        print(i)
        # time.sleep(0.1)
        ir_frame, rgb_frame, ir_frame_norm, pcd_frame, depth_frame = get_kinect_rgbd_frame(k4a)

        rgb_frame = rgb_frame[:, :, :3]

        ir_frame = np.expand_dims(ir_frame, axis = 2)
        depth_frame = np.expand_dims(depth_frame, axis = 2)

        res = np.zeros((rgb_frame.shape[0], rgb_frame.shape[1], 4))
        res[:, :, :3] = rgb_frame[:, :, :3]
        # res[:, :, -1:] = ir_frame
        res[:, :, -1:] = depth_frame

        with open(f"vtamp/demo{DEMO}/frames/frame" + str(i) + ".npy", 'wb') as f:
            np.save(f, res)
        # cv2.imshow('rgb', rgb_frame)
        cv2.waitKey(0)

def read_rgbd():
    pinhole_camera_intrinsic = np.array([[613.32427146,  0.,        633.94909346],
       [ 0.,        614.36077155, 363.33858573],
       [ 0.,          0.,          1.       ]])
    for i in range(10):
        if i > 1:
            frame = np.load("rgbd_frames/frame" + str(i) + ".npy")
            rgb = frame[:, :, :3].astype(np.uint8)
            # cv2.imshow('rgb', rgb)
            ir_frame = frame[:, :, -1:]
            ir_frame = np.clip(ir_frame, 0, 5e3) / 5e3  # Clip and normalize
            ir_frame = ir_frame.astype(np.float64)
            # cv2.imshow('depth', ir_frame)
            # # time.sleep(2)
            # cv2.waitKey(0)
            

            # rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb, ir_frame, convert_rgb_to_intensity = False)
            # pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, pinhole_camera_intrinsic)

            # # flip the orientation, so it looks upright, not upside-down
            # pcd.transform([[1,0,0,0],[0,-1,0,0],[0,0,-1,0],[0,0,0,1]])
            pcd_load = o3d.io.read_point_cloud("rgbd_frames/frame" + str(i) + ".npy")
            rgb = pcd_load[:, :, :3]
            o3d.visualization.draw_geometries([rgb])

            # o3d.draw_geometries([pcd])    # visualize the point cloud


def record_anchor_rgbd():
    ir_frame, rgb_frame, ir_frame_norm, pcd_frame, depth_frame = get_kinect_rgbd_frame(k4a)
    rgb = rgb_frame[:, :, :3]
    depth_frame = np.expand_dims(depth_frame, axis = 2)

    res = np.zeros((rgb_frame.shape[0], rgb_frame.shape[1], 4))
    res[:, :, :3] = rgb_frame[:, :, :3]
    # res[:, :, -1:] = ir_frame
    res[:, :, -1:] = depth_frame
    with open(f"vtamp/demo{DEMO}/anchor_frame.npy", 'wb') as f:
            np.save(f, res)

# os.makedirs(f"/home/lifanyu/tax3d/{DEMO}/", exist_ok=True)
os.makedirs(f"vtamp/demo{DEMO}/frames", exist_ok=True)

record_rgbd()
#read_rgbd()
# record_anchor_rgbd()
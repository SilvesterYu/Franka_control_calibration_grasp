from robot_controller import FrankaOSCController
import time
import numpy as np
import math
import os
import cv2
from scipy.spatial.transform import Rotation as R

from pyk4a import PyK4A


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

def record_input():
    pass

RESET_JOINTS = [ 0.94253151, 1.70307026, -1.46774482, 
                -2.24125153, 1.73320238, 1.610821, 2.02348159]
def reset_grasp(controller, k4a, trial, save_render = False):
    # release gripper
    controller.gripper_move_to(0.05)
    print("Releasing mug.")
    time.sleep(5)

    # home pose of gripper
    controller.reset(joint_positions = RESET_JOINTS)
    print("Robot reset.")
    time.sleep(8)

    # grasp the mug
    controller.gripper_move_to(0.002)
    print("Mug grasped.")
    breakpoint()
    time.sleep(5)

    # render
    if save_render:
        save_dir = os.path.expanduser(f"taskseg/inference/trial_{trial}/")
        os.makedirs(save_dir, exist_ok=True)
        ir_frame, rgb_frame, ir_frame_norm, pcd_frame, depth_frame = get_kinect_rgbd_frame(k4a)
        rgb_frame = rgb_frame[:, :, :3]
        depth_frame = np.expand_dims(depth_frame, axis = 2)
        # save trial render (rgb image, and depth image)
        height, width, layers = rgb_frame.shape
        cv2.imwrite(os.path.join(save_dir, "rgb.jpg"), rgb_frame)
        rgbd = np.concatenate([rgb_frame, depth_frame], axis=-1)
        np.save(os.path.join(save_dir, "rgbd.npy"), rgbd)
        print("Trial input captured.")

    print("Reset complete.")


k4a = PyK4A(device_id=0)
k4a.start()
trial = 0

if __name__ == "__main__":
    controller = FrankaOSCController(
        controller_type="OSC_POSE",
        visualizer=False)

    reset_grasp(controller, k4a, trial, True)
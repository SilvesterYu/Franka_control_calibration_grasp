import argparse
import pickle
import threading
import time
import os
from pathlib import Path
import time
import matplotlib.pyplot as plt
import numpy as np
import sys
import logging
import math
from autolab_core import RigidTransform

sys.path.append("/home/lifanyu/Documents/Github/Franka-Control-Calibration/frankapy") # change this. path to frankapy (I made some edits, but you can clone your own frankapy)
sys.path.append('/home/lifanyu/Documents/Github/Franka-Control-Calibration') # change this. path to controller

import frankapy
from frankapy import FrankaArm
from frankapy.franka_constants import FrankaConstants as FC
from frankapy.utils import franka_pose_to_rigid_transform

print('Starting robot')
fa = FrankaArm()

logging.getLogger().setLevel(logging.INFO)

PI = np.pi
EPS = np.finfo(float).eps * 4.0

from robot_controller.robot_controller import *

if __name__ == "__main__":
    controller = FrankaOSCController(
        controller_type="OSC_POSE",
        visualizer=False)
    
    # See https://frankaemika.github.io/docs/control_parameters.html
    JOINT_LIMITS_MIN = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
    JOINT_LIMITS_MAX = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]

    d = math.pi/8

    HOME_JOINTS = [0, 0, 0, -math.pi / 2, 0, math.pi / 2, math.pi / 4]

    # print the current RT of end effector, and joint positions of the robot
    read_RT(controller)
    ROT = np.array([
        [1, 0, 0],
        [0, -1, 0],
        [0, 0, -1]
    ])

    controller.reset(joint_positions = HOME_JOINTS)
    controller.move_to(np.array([ 0.55  , 0,  0.08]), use_rot = True, target_rot = ROT, duration = 4)
    time.sleep(3)
    controller.gripper_move_to(0.025)
    time.sleep(3)
    controller.move_to(np.array([ 0.45  , 0,  0.15]), use_rot = True, target_rot = ROT)
    controller.move_to(np.array([ 0.45  , 0,  0.02]), use_rot = True, target_rot = ROT)
    controller.gripper_move_to(0.08)
    controller.reset(joint_positions = HOME_JOINTS)

    
    # move to the top brick and put it aside
    controller.reset(joint_positions = HOME_JOINTS)
    controller.move_to(np.array([ 0.55  , 0,  0.08]), use_rot = True, target_rot = ROT, duration = 4)
    time.sleep(3)
    controller.gripper_move_to(0.025)
    time.sleep(3)
    controller.move_to(np.array([ 0.45  , 0,  0.15]), use_rot = True, target_rot = ROT)
    controller.move_to(np.array([ 0.45  , 0,  0.02]), use_rot = True, target_rot = ROT)
    controller.gripper_move_to(0.08)

    # reset, move to the second brick and move it aside
    controller.reset(joint_positions = HOME_JOINTS)
    controller.move_to(np.array([ 0.55  , 0,  0.05]), use_rot = True, target_rot = ROT, duration = 4)
    time.sleep(3)
    controller.gripper_move_to(0.025)
    time.sleep(3)
    controller.move_to(np.array([ 0.55  , 0.15,  0.15]), use_rot = True, target_rot = ROT)
    controller.move_to(np.array([ 0.55  , 0.15,  0.02]), use_rot = True, target_rot = ROT)
    controller.gripper_move_to(0.08)

    # reset, move the second brick on top of the first brick
    controller.reset(joint_positions = HOME_JOINTS)
    controller.move_to(np.array([ 0.55  , 0.15,  0.02]), use_rot = True, target_rot = ROT, duration = 4)
    time.sleep(3)
    controller.gripper_move_to(0.025)
    time.sleep(3)
    controller.move_to(np.array([ 0.45  , 0,  0.15]), use_rot = True, target_rot = ROT)
    controller.move_to(np.array([ 0.45  , 0,  0.05]), use_rot = True, target_rot = ROT)
    controller.gripper_move_to(0.08)

    # reset, move to the third brick and stack it on second brick
    controller.reset(joint_positions = HOME_JOINTS)
    controller.move_to(np.array([ 0.55  , 0,  0.02]), use_rot = True, target_rot = ROT, duration = 4)
    time.sleep(3)
    controller.gripper_move_to(0.025)
    time.sleep(3)
    controller.move_to(np.array([ 0.45  , 0,  0.15]), use_rot = True, target_rot = ROT)
    controller.move_to(np.array([ 0.45  , 0,  0.08]), use_rot = True, target_rot = ROT)
    controller.gripper_move_to(0.08)


    # back to ready pose
    controller.reset(joint_positions = HOME_JOINTS)
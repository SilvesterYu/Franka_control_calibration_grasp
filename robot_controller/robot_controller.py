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

sys.path.append("/home/lifanyu/Documents/Github/Franka_control_calibration_grasp/frankapy") # change this. path to frankapy (I made some edits, but you can clone your own frankapy)
sys.path.append('/home/lifanyu/Documents/Github/Franka_control_calibration_grasp') # change this. path to controller

import frankapy
from frankapy import FrankaArm
from frankapy.franka_constants import FrankaConstants as FC
from frankapy.utils import franka_pose_to_rigid_transform

print('Starting robot')
fa = FrankaArm()

logging.getLogger().setLevel(logging.INFO)

PI = np.pi
EPS = np.finfo(float).eps * 4.0

################################ Helper functions #################################

def quat2axisangle(quat):
    """
    Converts quaternion to axis-angle format.
    Returns a unit vector direction scaled by its angle in radians.

    Args:
        quat (np.array): (x,y,z,w) vec4 float angles

    Returns:
        np.array: (ax,ay,az) axis-angle exponential coordinates
    """
    # clip quaternion
    if quat[3] > 1.0:
        quat[3] = 1.0
    elif quat[3] < -1.0:
        quat[3] = -1.0

    den = np.sqrt(1.0 - quat[3] * quat[3])
    if math.isclose(den, 0.0):
        # This is (close to) a zero degree rotation, immediately return
        return np.zeros(3)

    return (quat[:3] * 2.0 * math.acos(quat[3])) / den

def axisangle2quat(vec):
    """
    Converts scaled axis-angle to quat.

    Args:
        vec (np.array): (ax,ay,az) axis-angle exponential coordinates

    Returns:
        np.array: (x,y,z,w) vec4 float angles
    """
    # Grab angle
    angle = np.linalg.norm(vec)

    # handle zero-rotation case
    if math.isclose(angle, 0.0):
        return np.array([0.0, 0.0, 0.0, 1.0])

    # make sure that axis is a unit vector
    axis = vec / angle

    q = np.zeros(4)
    q[3] = np.cos(angle / 2.0)
    q[:3] = axis * np.sin(angle / 2.0)
    return q

def quat2mat(quaternion):
    """
    Converts given quaternion to matrix.

    Args:
        quaternion (np.array): (x,y,z,w) vec4 float angles

    Returns:
        np.array: 3x3 rotation matrix
    """
    # awkward semantics for use with numba
    inds = np.array([3, 0, 1, 2])
    q = np.asarray(quaternion).copy().astype(np.float32)[inds]

    n = np.dot(q, q)
    if n < EPS:
        return np.identity(3)
    q *= math.sqrt(2.0 / n)
    q2 = np.outer(q, q)
    return np.array(
        [
            [1.0 - q2[2, 2] - q2[3, 3], q2[1, 2] - q2[3, 0], q2[1, 3] + q2[2, 0]],
            [q2[1, 2] + q2[3, 0], 1.0 - q2[1, 1] - q2[3, 3], q2[2, 3] - q2[1, 0]],
            [q2[1, 3] - q2[2, 0], q2[2, 3] + q2[1, 0], 1.0 - q2[1, 1] - q2[2, 2]],
        ]
    )

def pose2mat(pose):
    """
    Converts pose to homogeneous matrix.

    Args:
        pose (2-tuple): a (pos, orn) tuple where pos is vec3 float cartesian, and
            orn is vec4 float quaternion.

    Returns:
        np.array: 4x4 homogeneous matrix
    """
    homo_pose_mat = np.zeros((4, 4), dtype=np.float32)
    homo_pose_mat[:3, :3] = quat2mat(pose[1])
    homo_pose_mat[:3, 3] = np.array(pose[0], dtype=np.float32)
    homo_pose_mat[3, 3] = 1.0
    return homo_pose_mat

def mat2quat(rmat):
    """
    Converts given rotation matrix to quaternion.

    Args:
        rmat (np.array): 3x3 rotation matrix

    Returns:
        np.array: (x,y,z,w) float quaternion angles
    """
    M = np.asarray(rmat).astype(np.float32)[:3, :3]

    m00 = M[0, 0]
    m01 = M[0, 1]
    m02 = M[0, 2]
    m10 = M[1, 0]
    m11 = M[1, 1]
    m12 = M[1, 2]
    m20 = M[2, 0]
    m21 = M[2, 1]
    m22 = M[2, 2]
    # symmetric matrix K
    K = np.array(
        [
            [m00 - m11 - m22, np.float32(0.0), np.float32(0.0), np.float32(0.0)],
            [m01 + m10, m11 - m00 - m22, np.float32(0.0), np.float32(0.0)],
            [m02 + m20, m12 + m21, m22 - m00 - m11, np.float32(0.0)],
            [m21 - m12, m02 - m20, m10 - m01, m00 + m11 + m22],
        ]
    )
    K /= 3.0
    # quaternion is Eigen vector of K that corresponds to largest eigenvalue
    w, V = np.linalg.eigh(K)
    inds = np.array([3, 0, 1, 2])
    q1 = V[inds, np.argmax(w)]
    if q1[0] < 0.0:
        np.negative(q1, q1)
    inds = np.array([1, 2, 3, 0])
    return q1[inds]

def quat_multiply(quaternion1, quaternion0):
    """
    Return multiplication of two quaternions (q1 * q0).

    E.g.:
    >>> q = quat_multiply([1, -2, 3, 4], [-5, 6, 7, 8])
    >>> np.allclose(q, [-44, -14, 48, 28])
    True

    Args:
        quaternion1 (np.array): (x,y,z,w) quaternion
        quaternion0 (np.array): (x,y,z,w) quaternion

    Returns:
        np.array: (x,y,z,w) multiplied quaternion
    """
    x0, y0, z0, w0 = quaternion0
    x1, y1, z1, w1 = quaternion1
    return np.array(
        (
            x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
            -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
            x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
            -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
        ),
        dtype=np.float32,
    )

def quat_conjugate(quaternion):
    """
    Return conjugate of quaternion.

    E.g.:
    >>> q0 = random_quaternion()
    >>> q1 = quat_conjugate(q0)
    >>> q1[3] == q0[3] and all(q1[:3] == -q0[:3])
    True

    Args:
        quaternion (np.array): (x,y,z,w) quaternion

    Returns:
        np.array: (x,y,z,w) quaternion conjugate
    """
    return np.array(
        (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]),
        dtype=np.float32,
    )


def quat_inverse(quaternion):
    """
    Return inverse of quaternion.

    E.g.:
    >>> q0 = random_quaternion()
    >>> q1 = quat_inverse(q0)
    >>> np.allclose(quat_multiply(q0, q1), [0, 0, 0, 1])
    True

    Args:
        quaternion (np.array): (x,y,z,w) quaternion

    Returns:
        np.array: (x,y,z,w) quaternion inverse
    """
    return quat_conjugate(quaternion) / np.dot(quaternion, quaternion)

def quat_distance(quaternion1, quaternion0):
    """
    Returns distance between two quaternions, such that distance * quaternion0 = quaternion1

    Args:
        quaternion1 (np.array): (x,y,z,w) quaternion
        quaternion0 (np.array): (x,y,z,w) quaternion

    Returns:
        np.array: (x,y,z,w) quaternion distance
    """
    return quat_multiply(quaternion1, quat_inverse(quaternion0))

def estimate_tag_pose(finger_pose):
    """
    Estimate the tag pose given the gripper pose by applying the gripper-to-tag transformation.

    Args:
        finger_pose (eef_pose): 4x4 transformation matrix from gripper to robot base
    Returns:
        hand_pose: 4x4 transformation matrix from hand to robot base
        tag_pose: 4x4 transformation matrix from tag to robot base
    """
    from scipy.spatial.transform import Rotation

    # Estimate the hand pose
    # finger_to_hand obtained from the product manual: 
    # [https://download.franka.de/documents/220010_Product%20Manual_Franka%20Hand_1.2_EN.pdf]
    finger_to_hand = np.array([
        [0.707,  0.707, 0, 0],
        [-0.707, 0.707, 0, 0],
        [0, 0, 1, 0.1034],
        [0, 0, 0, 1],
    ])
    finger_to_hand = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0.1034],
        [0, 0, 0, 1],
    ])
    hand_to_finger = np.linalg.inv(finger_to_hand)
    print("hand to finger", hand_to_finger)
    hand_pose = np.dot(finger_pose, hand_to_finger)

    t_tag_to_hand = np.array([0.048914, 0.0275, 0.00753])
    # R_tag_to_hand = Rotation.from_quat([0.5, -0.5, 0.5, -0.5])
    R_tag_to_hand = Rotation.from_quat([0, 0, 0, 1])
    tag_to_hand = np.eye(4)
    tag_to_hand[:3, :3] = R_tag_to_hand.as_matrix()
    tag_to_hand[:3, 3] = t_tag_to_hand

    tag_pose = np.dot(hand_pose, tag_to_hand)
    
    return hand_pose, tag_pose

def compute_errors(pose_1, pose_2):

    pose_a = (
        pose_1[:3]
        + quat2axisangle(np.array(pose_1[3:]).flatten()).tolist()
    )
    pose_b = (
        pose_2[:3]
        + quat2axisangle(np.array(pose_2[3:]).flatten()).tolist()
    )
    return np.abs(np.array(pose_a) - np.array(pose_b))

def rpy2mat(rot_x, rot_y, rot_z):
    """Create a rotation matrix from Euler angles."""
    rx = np.array([
        [1, 0, 0],
        [0, np.cos(rot_x), -np.sin(rot_x)],
        [0, np.sin(rot_x), np.cos(rot_x)]
    ])
    ry = np.array([
        [np.cos(rot_y), 0, np.sin(rot_y)],
        [0, 1, 0],
        [-np.sin(rot_y), 0, np.cos(rot_y)]
    ])
    rz = np.array([
        [np.cos(rot_z), -np.sin(rot_z), 0],
        [np.sin(rot_z), np.cos(rot_z), 0],
        [0, 0, 1]
    ])
    return rz @ ry @ rx

# Read the robot's current RT
def read_RT(controller):
    print("FC", FC.HOME_POSE)
    eef_pose = controller.eef_pose
    hand_pose, tag_pose = estimate_tag_pose(eef_pose)
    print(f"eef pos: {eef_pose[:3, 3]}")
    print(f"hand pos: {hand_pose[:3, 3]}")
    print(f"Tag pos: {tag_pose[:3, 3]}")

    joint_positions = controller.joint_positions
    print(f"Joint positions: {joint_positions}")

    ee_rot_pos = controller.eef_rot_and_pos

    print(f"End effector rotation and position: {ee_rot_pos}")

################################### The controller class ########################################
class FrankaOSCController():
    def __init__(self,
                 interface_cfg="charmander.yml",
                 controller_type="OSC_POSE",
                 # change this to your own path
                 controller_cfg="/home/lifanyu/Documents/Github/Franka_control_calibration_grasp/frankapy/tuned-osc-yaw-controller.yml",
                 visualizer=False):
        
        self.controller_cfg = controller_cfg

        self.reset_joint_positions = [
            -0.5493463,
            0.18639661,
            0.04967389,
            -1.92004654,
            -0.01182675,
            2.10698001,
            0.27106661]
        
    def reset(self, joint_positions=None, grasp=True, ignore_virtual_walls=True, duration = 3):
        joint_positions = joint_positions if joint_positions is not None else FC.HOME_JOINTS
        fa.goto_joints(joint_positions, ignore_virtual_walls=ignore_virtual_walls, duration=duration)
        if grasp == False:
            fa.open_gripper()

    # Go to Franka arm's official reset pose
    def reset_franka(self, grasp = False):
        if grasp == False:
            fa.open_gripper()
        fa.goto_pose(FC.HOME_POSE)
        

    def reset_pose(self):
        fa.reset_pose()

    # move to the target quat or XYZ coordinate + Rotation
    def move_to(self, 
                target_pos,
                target_quat=None,
                target_delta_axis_angle=None,
                grasp=True,
                use_rot = False,
                target_rot = None,
                duration = 3,
                num_steps=40,
                num_additional_steps=20):
        
        if target_quat is not None:
            pass
        if target_delta_axis_angle is not None:
            current_axis_angle = self.eef_axis_angle
            target_axis_angle = current_axis_angle + target_delta_axis_angle
            target_quat = axisangle2quat(target_axis_angle)
        elif use_rot == False:
            raise ValueError("Either target_quat or target_delta_axis_angle must be specified")

        if use_rot == True:
            TARGET_ROT = target_rot
        else:
            TARGET_ROT = quat2mat(target_quat)
        TARGET_POSE = RigidTransform(rotation=TARGET_ROT, 
                                     translation=target_pos,
                                     from_frame='franka_tool', to_frame='world')

        fa.goto_pose(TARGET_POSE, duration = duration)
        if grasp == False:
            fa.open_gripper()

    # move by a certain delta position XYZ or delta axis angle
    def move_by(self, 
                target_delta_pos=np.zeros(3), 
                target_delta_axis_angle=np.zeros(3),
                grasp=True,
                num_steps=40,
                num_additional_steps=20,
                duration = 3):
        
        current_ee_pose = self.eef_pose
        current_pos = current_ee_pose[:3, 3:]
        current_rot = current_ee_pose[:3, :3]
        current_quat = mat2quat(current_rot)
        current_axis_angle = quat2axisangle(current_quat)

        target_pos = np.array(target_delta_pos).reshape(3, 1) + current_pos
        target_axis_angle = np.array(target_delta_axis_angle) + current_axis_angle

        # logger.info(f"Before conversion {target_axis_angle}")
        target_quat = axisangle2quat(target_axis_angle)
        target_pose = target_pos.flatten().tolist() + target_quat.flatten().tolist()

        if np.dot(target_quat, current_quat) < 0.0:
            current_quat = -current_quat
        target_axis_angle = quat2axisangle(target_quat)
        # logger.info(f"After conversion {target_axis_angle}")
        current_axis_angle = quat2axisangle(current_quat)

        start_pose = current_pos.flatten().tolist() + current_quat.flatten().tolist()
        
        # print("\nMoving by target delta pose:\n", target_delta_pos)
        self.move_to(target_pos, target_quat, target_delta_axis_angle=target_delta_axis_angle, grasp=grasp, num_steps=num_steps, num_additional_steps=num_additional_steps, duration=duration)

    # move the gripper to a certain width
    # passing block = True will block the code until the gripper reaches the target width
    def gripper_move_to(self, width, block = False):
        if width == 0.08:
            # grasp = True indicates that the gripper is in a grasping state, and actively respond to commands that open it
            fa.goto_fripper(width = width, block = block, grasp = True)
        else:

            fa.goto_gripper(width=width)
    
    def update_controller_config(self, controller_cfg):
        self.controller_cfg = controller_cfg
    
    @property
    def eef_axis_angle(self):
        rot = self.eef_rot_and_pos[0]
        quat = mat2quat(rot)
        return quat2axisangle(quat)

    @property
    def eef_pose(self):
        # return self.robot_interface.last_eef_pose
        current_ee_pose = np.array(fa._state_client._get_current_robot_state().robot_state.O_T_EE).reshape(4, 4).transpose()
        print(current_ee_pose)
        return current_ee_pose
    
    def get_eef_pose(self):
        return np.array(fa._state_client._get_current_robot_state().robot_state.O_T_EE).reshape(4, 4).transpose()
    
    @property
    def eef_rot_and_pos(self):
        # return self.robot_interface.last_eef_rot_and_pos
        current_ee_pose = fa.get_pose()
        return current_ee_pose.rotation, current_ee_pose.translation

    @property
    def joint_positions(self):
        # return self.robot_interface.last_q
        return fa.get_joints()
    
    @property
    def gripper_position(self):
        # return the current gripper position (open width)
        np.array(fa._state_client._get_current_robot_state().robot_state.O_T_EE).reshape(4, 4).transpose()


################################## Movements #################################
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

    # usage:
    controller.reset_franka()
    controller.reset(joint_positions = HOME_JOINTS)
    # controller.reset(joint_positions = FC.READY_JOINTS)
    # controller.move_by(np.array([0.1, 0, 0]), np.array([0, 0, 0]), num_steps=40, num_additional_steps=10)
    # controller.move_by(np.array([0.0, 0.1, 0]), np.array([0, 0, 0]), num_steps=40, num_additional_steps=10)
    # controller.move_by(np.array([0.0, 0, 0.1]), np.array([0, 0, 0]), num_steps=40, num_additional_steps=10)
    # controller.reset_franka()
    # controller.reset(joint_positions = HOME_JOINTS)

    # Another example usage of robot moving a brick




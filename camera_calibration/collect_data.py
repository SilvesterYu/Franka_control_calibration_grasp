import sys
import os
import time
import numpy as np
import math
import pickle
import cv2
from tqdm import tqdm
from scipy.spatial.transform import Rotation
from pyk4a import PyK4A
from pyk4a.calibration import CalibrationType

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)

# import the controller as well as helper functions
from robot_controller.robot_controller import *
from marker_detection import get_kinect_ir_frame, detect_aruco_markers, estimate_transformation, get_kinect_rgb_frame

print("Start Data Collection")

def move_robot_and_record_data(
        cam_id, 
        num_movements=3, 
        debug=False,
        initial_joint_positions=None):
    """
    Move the robot to random poses and record the necessary data.
    """

    # Initialize the robot
    robot = FrankaOSCController()
    
    # # Initialize the robot
    # robot = FrankaOSCController(
    #     tip_offset=np.zeros(3),     # Set the default to 0 to disable accounting for the tip
    # )

    # Initialize the camera
    k4a = PyK4A(device_id=cam_id)
    k4a.start()
    # camera_matrix = k4a.calibration.get_camera_matrix(CalibrationType.DEPTH)
    # dist_coeffs = k4a.calibration.get_distortion_coefficients(CalibrationType.DEPTH)
    camera_matrix = np.array([[613.32427146,  0.,        633.94909346],
        [ 0.,        614.36077155, 363.33858573],
        [ 0.,          0.,          1.       ]])
    dist_coeffs = np.array([[ 0.09547761, -0.06461896, -0.00039569, -0.00243461, 0.02172413]])

    data = []
    for i in range(len(initial_joint_positions)):
        this_initial_joint_positions = initial_joint_positions[i]
        for _ in tqdm(range(num_movements)):
            print("point ", i)
            print(f"\nRecorded {len(data)} data points.")
            # Generate a random target delta pose
            random_delta_pos = [np.random.uniform(-0.06, 0.06, size=(3,))]
            random_delta_axis_angle = [np.random.uniform(-0.3, 0.3, size=(3,))]
            robot.reset(joint_positions=this_initial_joint_positions)
            robot.move_by(random_delta_pos, random_delta_axis_angle, num_steps=40, num_additional_steps=30)

            # Get current pose of the robot 
            gripper_pose = robot.eef_pose
            print(f"Gripper pos: {gripper_pose[:3, 3]}")

            # Capture IR frame from Kinect
            ir_frame = get_kinect_rgb_frame(k4a)
            if ir_frame is not None:
                # Detect ArUco markers and get visualization
                corners, ids = detect_aruco_markers(ir_frame, debug=debug)

                # Estimate transformation if marker is detected
                if ids is not None and len(ids) > 0:
                    transform_matrix = estimate_transformation(corners, ids, camera_matrix, dist_coeffs)
                    if transform_matrix is not None:
                        data.append((
                            gripper_pose,       # gripper pose in base
                            transform_matrix    # tag pose in camera
                        ))
            else:
                print("\033[91m" + "No IR frame captured." + "\033[0m")
    
    print(f"Recorded {len(data)} data points.")
    # Save data
    os.makedirs("calibration_data/", exist_ok=True)
    filepath = f"calibration_data//cam{cam_id}_data.pkl"
    with open(f"calibration_data//cam{cam_id}_data.pkl", "wb") as f:
        pickle.dump(data, f)
    print(filepath)
    return filepath

def main():
    # 0: main
    # 2: front
    # 1: back

    cam_id = 0


    initial_joint_positions = {
        
        # left
        0: [
            
            # [ 1.01613354,  0.69271861, -1.13224949, -2.29435196,  0.41520166,  2.53452068,  1.94347663],
            # [0.73684042,  1.00086083, -1.22985438, -1.9849199,   0.75299306,  2.30539459,  1.56744786],
            # [ 1.195152,    0.93037208, -1.16595389, -2.13945461,  0.87474924,  2.2678842,  1.88733302],

            #  [ 0.49731168,  0.89957801, -1.33366195, -1.76468232,  0.57594931,  2.12555905,  1.66851225],
              [ 1.20726028,  0.87485253, -1.45102114,-1.99436911,  0.59290017, 2.05114472,  1.83426653],
               [ 0.708285,    0.80053989, -1.17216022, -2.11689471,  0.72960887,  2.41734635,  1.56737142],
            #  [ 0.85161842,  0.924047,   -1.18878356, -1.8897045,   0.7838417,  2.27904548,  1.72914356],
    [ 1.41885441,  1.02131318, -1.51016132, -2.32741208,  0.92510411,  2.17205265,  1.74765601],



            # [ 0.26071136,  0.86780072, -1.00729773, -1.93948225,  0.94714757,  2.4701816,  1.06387623],
            # [ 0.3399307,   1.05193124, -1.04214905, -1.59355958,  0.78800553,  2.12766938,  1.61683901],
            # [ 0.35282318,  0.93495763, -1.18115077, -2.21104504,  1.3165264,   2.47844767,  0.31374606]
        ],

        # back
        2: [
            [ 0.80303131,  1.08058283, -1.35028133, -2.19741646,  0.79895496,  2.11702867,  2.42144591],
            [ 0.66526714,  1.11202762, -1.40838012, -2.15410958,  0.76629778,  2.09221551,  2.4208291 ],
            [ 0.79361711,  1.11210172, -1.30672449, -2.07275462,  0.77168969,  2.09209669,  2.47623681],
            [ 0.75931009,  1.14560888, -1.27707491, -1.77922318,  0.78018166,  1.91958368,  2.71712   ],
            # [ 0.36041067,  1.20331984, -1.16795492, -1.47667899,  0.67356189,  1.92051929,  2.70286353],
            # [ 0.90789889,  1.22442351, -1.35075966, -1.81845327,  0.94880109,  1.91797615,  2.74570475],
            # [ 1.10956595,  1.2365553,  -1.3831393,  -1.82290093,  0.98423194,  1.73020111,  2.85006708],
            [ 0.77653032,  1.38327789, -1.38831455, -1.6420015,   1.00504129,  1.73162375,  2.82685177],
            # [ 0.34712354,  1.39607519, -1.31853838, -1.52910317,  0.91590294,  1.8914474,  2.5736011 ],
            # [ 0.36924249,  1.0123573,  -1.25192504, -1.97114164,  0.86342411,  2.23594519,  2.19599842],
            # [ 0.48447817,  1.02558934, -1.15981421, -1.82954647,  0.87733619,  2.23710183,  2.54698531],
            [ 1.00849527,  1.37234158, -1.46545473, -1.84913419,  0.82744496,  1.62359741,  2.71717232]
            ],


        1: [
            [-0.31439241,  0.53667096, -0.10021914, -1.82017653,  0.53242127,  2.38310771, -0.8403276 ],
            [-0.40712642,  0.05387728,  0.11946909, -2.23737594,  0.49878166,  2.39964904, -0.45635229],
            # [-1.29999853,  0.87133742,  1.00803161, -2.38818703, -0.38655018,  3.229907,  0.03499247],
            # [-1.24523068,  1.29032512,  1.20539191, -1.65760581, -0.38715188,  2.5740232, -0.33458953],
            [-1.3899861,   1.5340643,   1.28052274, -1.63285435, -0.64254028,  2.45213039, -0.35267662],
            [-1.44476268,  1.25879042,  1.06387623],
            [ 0.3399307,   1.05193124, -1.04214905, -1.59355958,  0.78800553,  2.12766938,  1.61683901],
            [ 0.35282318,  0.93495763, -1.18115077, -2.21104504,  1.3165264,   2.47844767,  0.31374606],
            # [-1.37319121,  0.8762792,   1.20708171, -2.05247011, -0.62970937,  2.65916116,  0.27329053]
        ]
        # 0: [-0.86812917, 0.36391594, 0.25352557, -1.92162717, 0.12602475, 2.1308299, -1.50102163],
        # 1: [-0.83424677, 0.42084166, 0.2774182, -1.97982254, -0.1749291, 2.40231471, 0.27310384],
        # 2: [-0.81592058, 0.39429853, 0.29050235, -1.88333403, -0.17686262, 2.28619198, 1.98916667],
        # 3: [-0.85456277, 0.36942704, 0.38232294, -1.88742087, -0.45677587, 2.19400042, -2.88310376]
    }[cam_id]
    
    # Perform the movements and record data
    move_robot_and_record_data(
        cam_id=cam_id, num_movements=10, debug=False, 
        initial_joint_positions=initial_joint_positions)
    

if __name__ == "__main__":
    main()
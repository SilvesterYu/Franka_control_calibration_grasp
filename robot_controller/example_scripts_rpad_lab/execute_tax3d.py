from robot_controller import FrankaOSCController
import time
import numpy as np
import math
from scipy.spatial.transform import Rotation as R

from pyk4a import PyK4A


CLOTH_START_POS = np.array([ 0.45, -0.42,  0.45])
CLOTH_START_ROT = np.array([[1.0, 0.0, 0.0],
                                [0.0, -1.0, 0.0],
                                [0.0, 0.0, -1.0]])

def get_kinect_rgbd_frame(device):
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
                depth_frame = capture.transformed_depth
                rgb_frame = capture.color

                # print(pcd_frame.shape, ir_frame.shape)
                # print("successful capture")
                print("success: ", rgb_frame.shape, depth_frame.shape)
                rgb_frame = rgb_frame[:, :, :3]
                depth_frame = np.expand_dims(depth_frame, axis=2)
                rgbd = np.concatenate([rgb_frame, depth_frame], axis=-1)
                return rgbd
        except:
            time.sleep(0.1)
            # print("Failed to capture IR frame.")
    else:
        # print("Failed to capture IR frame after 20 attempts.")
        return None
    

def reset_and_grasp(controller, capture_device):
    # Initially, release gripper and reset
    controller.gripper_move_to(0.08)

    # RESET POSE OUT OF VIEW OF CAMERA TO GET CAPTURE OF ONLY ANCHOR
    HOME_JOINTS = [-2.8498962,  -0.50635254,  1.81945124, -1.63473409,  0.49385608,  1.79495532, -0.35049537]
    # HOME_JOINTS = [-2.16983479, -0.98964959,  0.88810447, -1.60529673,  0.86708144,  0.9975407, -0.21358609]

    controller.reset(joint_positions = HOME_JOINTS)
    print("Robot reset.")
    pre_grasp_position = np.array([ 0.34054935, -0.44315122,  0.45])
    pre_grasp_rotation = np.array([[ 0.99880188,  0.04509791,  0.01848585],
       [ 0.04587327, -0.99797171, -0.04391925],
       [ 0.01646769,  0.04471463, -0.99886404]])
    controller.move_to(pre_grasp_position, use_rot = True, target_rot = pre_grasp_rotation, duration = 5)
    print("Robot pre-grasp.")

    time.sleep(3)
    rgbd = get_kinect_rgbd_frame(capture_device)
    with open(f"/home/lifanyu/tax3d/experiment/anchor_frame.npy", 'wb') as f:
            np.save(f, rgbd)
    print("Anchor captured.")


    # CLOTH STARTING POSE
    target_position = np.array([ 0.40, -0.42,  0.45])
    target_rotation = np.array([[1.0, 0.0, 0.0],
                                [0.0, -1.0, 0.0],
                                [0.0, 0.0, -1.0]])
    
    at_grasp = False
    for _ in range(3):
        controller.move_to(CLOTH_START_POS, use_rot = True, target_rot = CLOTH_START_ROT, duration = 10)
        ee_rot_pos = controller.get_eef_pose()
        ee_pos = ee_rot_pos[0:3, 3]
        print(ee_pos)
        at_grasp = np.isclose(ee_pos, CLOTH_START_POS, atol=1e-3).all()
        if at_grasp:
             break
        else:
             print("Cloth grasp not reached, trying again...")
    if at_grasp:
         print("Robot cloth grasp.")
    else:
         print("Cloth grasp not reached after maximum attempts. Quitting...")
         quit()

    #controller.move_to(target_position, use_rot = True, target_rot = target_rotation, duration = 5)
    #print("Robot cloth grasp.")
    time.sleep(2)
    controller.gripper_move_to(0.002) # optimal parameter for cloth grasp

    time.sleep(5) # wait, just in case the cloth has some slight movement
    rgbd = get_kinect_rgbd_frame(capture_device)
    with open(f"/home/lifanyu/tax3d/experiment/action_frame.npy", 'wb') as f:
            np.save(f, rgbd)
    print("Action captured.")
    # print("Cloth captured.")
    


def place_cloth(controller, r, t, offset):
    """
        R is axis-angle rotation, t is translation vector.
    """
    # assuming the cloth has already been grasped, reset to start just in case
    # target_position = np.array([ 0.35, -0.42,  0.45])
    # target_rotation = np.array([[1.0, 0.0, 0.0],
    #                             [0.0, -1.0, 0.0],
    #                             [0.0, 0.0, -1.0]])
    # controller.move_to(target_position, use_rot = True, target_rot = target_rotation, duration = 5)
    at_grasp = False
    for _ in range(3):
        controller.move_to(CLOTH_START_POS, use_rot = True, target_rot = CLOTH_START_ROT, duration = 10)
        ee_rot_pos = controller.get_eef_pose()
        ee_pos = ee_rot_pos[0:3, 3]
        print(ee_pos)
        at_grasp = np.isclose(ee_pos, CLOTH_START_POS, atol=1e-3).all()
        if at_grasp:
             break
        else:
             print("Cloth grasp not reached, trying again...")
    if at_grasp:
         print("Robot cloth grasp.")
    else:
         print("Cloth grasp not reached after maximum attempts. Quitting...")
         quit()

    time.sleep(2)
    # compute target pose
    ee_pos = controller.get_eef_pose()
    r_current = ee_pos[0:3, 0:3]
    t_current = ee_pos[0:3, 3]
    target_r = r @ r_current
    # target_r = r_current
    target_t1 = t_current + t + offset#+ t#  + [0.08, 0.0, 0.0]
    target_t2 = t_current + t
    # print(target_r)
    
    print("target1: ", target_t1)
    breakpoint()
    controller.move_to(target_t1, use_rot = True, target_rot = target_r, duration = 10)
    time.sleep(1)
    breakpoint()
    controller.move_to(target_t2, use_rot = True, target_rot = target_r, duration = 5)
    # controller.move_by(target_delta_pos = t, target_delta_axis_angle = r, duration = 10)
    print("Robot cloth placement.")
    ee_rot_pos = controller.get_eef_pose()
    r_current = ee_pos[0:3, 0:3]
    t_current = ee_pos[0:3, 3]
    print(t_current)
    breakpoint()
    controller.gripper_move_to(0.03)


if __name__ == "__main__":
    controller = FrankaOSCController(
        controller_type="OSC_POSE",
        visualizer=False)
    k4a = PyK4A(device_id=0)
    k4a.start()
    
    
    
    HOME_JOINTS = [-2.8498962,  -0.50635254,  1.81945124, -1.63473409,  0.49385608,  1.79495532, -0.35049537]
    # HOME_JOINTS = [-2.16983479, -0.98964959,  0.88810447, -1.60529673,  0.86708144,  0.9975407, -0.21358609]
    
    # controller.reset(joint_positions = HOME_JOINTS)



    # controller.reset(joint_positions = HOME_JOINTS)
    # # time.sleep(10)
    # # controller.gripper_move_to(0.002)
    # quit()

    r = np.array(
[[-0.29128352, -0.95626783, -0.02656571],
 [ 0.95629487, -0.2918095,   0.0186368 ],
 [-0.0255739,  -0.01997606,  0.99947333]] 
 )
    t = np.array(
 [-0.06503803,  0.23664024,  0.04982707] 
)
    offset = np.array(
[0.0951664,  0.07309826, 0]
    )

    # t_adjust = R.from_euler('xyz', [0, 0, 0], degrees=True)
    # t = t_adjust.apply(t)

    # reset_and_grasp(controller, k4a)
    place_cloth(controller, r, t, offset)
    # # Find a good reset pose for the cloth

    # TODO: for placement, rotate first, and do the translation


    # # Initially, release gripper and reset
    # controller.gripper_move_to(0.08)
    # time.sleep(2)

    # # Change this initial reset position as needed
    # # HOME_JOINTS = [-1.46929639,  0.19567124,  0.8190383,  -1.63386635, -0.07106318,  1.70830441,  0.28554653] # center - left of workspace
    # # HOME_JOINTS = [0, 0, 0, -math.pi / 2, 0, math.pi / 2, math.pi / 4] # center of workspace



    # HOME_JOINTS = [-2.8498962,  -0.50635254,  1.81945124, -1.63473409,  0.49385608,  1.79495532, -0.35049537]
    # controller.reset(joint_positions = HOME_JOINTS)
    # print("Robot reset.")
    # # controller.move_to(np.array([0.54, -0.13, 0.4]), use_rot = True, target_rot = np.array([[1, 0, 0],        [0, -1, 0],        [0, 0, -1]]), duration = 3)





    # # CLOTH STARTING POSE
    # target_position = np.array([ 0.42562306, -0.47475452,  0.46770836])
    # target_rotation = np.array([[ 0.99955406, -0.0294749 ,  0.00191158],
    #     [-0.02950079, -0.99943746,  0.01533709],
    #     [ 0.00145845, -0.01538665, -0.99988055]])
    # controller.move_to(target_position, use_rot = True, target_rot = target_rotation, duration = 3)
    # print("Robot moved to starting pose.")

    # time.sleep(3)
    # controller.gripper_move_to(0.002) # optimal parameter for cloth grasp
    # print("Cloth grasped.")

    # breakpoint()

    # controller.move_by(target_delta_pos = np.array([0, 0, 0.1]))
    # controller.move_by(target_delta_pos = np.array([0, 0.1, 0]))
    # controller.move_by(target_delta_pos = np.array([0.1, 0, 0]))


    # READ 
    quit()


    # ROBOT GRASPING THE CLOTH AT START TIME
    # to close the fripper, set it to zero or to a small number. The unit is meter
    controller.gripper_move_to(0.002) # 0.002 is the optimal parameter
    # change this accordingly
    time.sleep(3)

    # ### TODO: fill these in from the model output. These should be in world coordinates
    # target_positions = [
    #     [0.25, -0.4, 0.33],
    #     [0.35, -0.3, 0.4],
    #     [0.45, -0.2, 0.45],
    # ]
    target_positions = [
        [0.4, -0.41, 0.33],
        [0.399, -0.335, 0.341],
        [0.4085, -0.276, 0.348],
        [0.419, -0.225, 0.346],
        [0.436, -0.177, 0.321]
    ]
    for item in target_positions:
        item[1] -= 0.05
    target_rotations = [
        [[1, 0, 0],        [0, -1, 0],        [0, 0, -1]],
        [[1, 0, 0],        [0, -1, 0],        [0, 0, -1]],
        [[1, 0, 0],        [0, -1, 0],        [0, 0, -1]],
        [[1, 0, 0],        [0, -1, 0],        [0, 0, -1]],
        [[1, 0, 0],        [0, -1, 0],        [0, 0, -1]]
    ]
    
    # ROBOT MOVING THE CLOTH
    # upright default rotation is np.array([[1, 0, 0],        [0, -1, 0],        [0, 0, -1]])
    for i in range(len(target_positions)):
        pos = np.array(target_positions[i])
        rot = np.array(target_rotations[i])
        # unit for duration is seconds. Change it to make the robot move slower or quicker
        controller.move_to(np.array(pos), use_rot = True, target_rot = np.array(rot), duration = 3)

    # ROBOT RELEASING THE CLOTH. This is the max width of gripper (when it's open)
    controller.gripper_move_to(0.08)
    
    # controller.reset(joint_positions = HOME_JOINTS)

    # If the robot fails to quickly release the cloth, uncomment the below lines
    # time.sleep(5)
    # controller.gripper_move_to(0.08)

    
"""

End effector rotation and position: (array([[ 0.99955406, -0.0294749 ,  0.00191158],
       [-0.02950079, -0.99943746,  0.01533709],
       [ 0.00145845, -0.01538665, -0.99988055]]), array([ 0.42562306, -0.47475452,  0.46770836]))


"""

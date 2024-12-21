import open3d as o3d
import numpy as np
import copy
import os
import sys
import json
from scipy.spatial.transform import Rotation

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(current_dir)

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
extrinsics_dirname = os.path.join(parent_dir, "camera_calibration", "calibration_results")

# alignment_dirname = os.path.join(parent_dir, "camera_alignment/alignment_results")
# alignment_path = os.path.join(alignment_dirname, "camera_alignments_matrices.npz")

# TODO: Uncomment this if you are done with camera alignment and do not wish to accidentally to overwrite the alignment results
# if os.path.exists(alignment_path):
#     raise Exception(f"The camera alignment file '{alignment_path}' exists. If you want to re-compute the alignments and overwrite it, delete it first.")

from camera_alignment.pointcloud_env import PointcloudEnv
from camera_alignment.utils import *

# Load pointcloud processing envieronment
env = PointcloudEnv(
    camera_indices=[1],
    camera_extrinsics_dir=extrinsics_dirname,
    # camera_alignment argument is used ONLY for multi-camera setting. Comment it out for single camera setting or when cameras are not aligned yet
    # camera_alignments=alignment_path,
    camera_alignments=False # set to False when starting from sratch, when cameras are not aligned yet
)

# colors = [np.random.rand(3) for i in range(100)]
colors = [
    np.array([0.8, 0.2, 0.2]),
    np.array([0.2, 0.8, 0.2]),
    np.array([0.2, 0.2, 0.8])
    ]

# Visualize initial point clouds
import open3d as o3d
import numpy as np

initial_pcd = np.load("initial_pcd.npy")
initial_pcd_seg = np.load("initial_pcd_seg.npy")

object0_pcd = initial_pcd[initial_pcd_seg == 0]
object1_pcd = initial_pcd[initial_pcd_seg == 1]
object2_pcd = initial_pcd[initial_pcd_seg == 2]

np.save("object2_pcd.npy", object2_pcd)

# python3 contact_graspnet/inference.py --np_path=/home/lifanyu/Documents/Github/Franka_control_calibration_grasp/demo_collection/object0_pcd.npy \
#                                      --forward_passes=5 \
#                                      --z_range=[0.2,1.1]

# python3 contact_graspnet/inference.py \
#        --np_path=test_data/7.npy \
#        --local_regions --filter_grasps

object_pcd_list = [object0_pcd, object1_pcd, object2_pcd]
object_o3d_pcd_list = [o3d.geometry.PointCloud(o3d.utility.Vector3dVector(object_pcd)) for object_pcd in object_pcd_list]

objects = np.load("objects.npy")
transforms = np.load("transforms.npy")

print(initial_pcd.shape)
print(initial_pcd_seg.shape)
print(transforms[0])

pcd = o3d.geometry.PointCloud()
# for cam_id in env.camera_indices:
#     processed_pcd = env.get_single_pcd(cam_id)

#     # can crop it according to the specific task
#     # processed_pcd = crop(processed_pcd, 0.2, 0.9, -0.5, 0.5, -0.12, 0.5)

#     # processed_pcd = processed_pcd
        
#     processed_pcd.paint_uniform_color(colors[cam_id])
#     pcd += processed_pcd

for object_id in objects:
    object_pcd = object_o3d_pcd_list[object_id]
    # object_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(object_pcd))
    object_pcd.paint_uniform_color(colors[object_id])
    pcd += object_pcd

pcd = env.transform_raw_pcd(pcd, 1)
o3d.visualization.draw_geometries([pcd])

# transform objects
object_o3d_transformed_list = [None] * len(objects)
for object_id in objects:
    object_pcd = object_o3d_pcd_list[object_id]
    transform = transforms[object_id]
    object_pcd.transform(transform)
    object_o3d_transformed_list[object_id] = object_pcd

pcd = o3d.geometry.PointCloud()
for object_id in objects:
    object_pcd = object_o3d_transformed_list[object_id]
    object_pcd.paint_uniform_color(colors[object_id])
    pcd += object_pcd

pcd = env.transform_raw_pcd(pcd, 1)
o3d.visualization.draw_geometries([pcd])

test = np.load("/home/lifanyu/Documents/Github/contact_graspnet/test_data/0.npy", allow_pickle=True)
print(test.shape)

#### You can save the pcd here (Note from Lifan)

import open3d as o3d
import numpy as np
import copy
import os
import sys
import json
from scipy.spatial.transform import Rotation

from pointcloud_env import PointcloudEnv
from utils import *

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
extrinsics_dirname = os.path.join(parent_dir, "camera_calibration", "calibration_results")

alignment_dirname = os.path.join(parent_dir, "camera_calibration", "alignment_results")
alignment_path = os.path.join(alignment_dirname, "camera_alignments_matrices.npz")

# TODO: Uncomment this if you are done with camera alignment and do not wish to accidentally to overwrite the alignment results
if os.path.exists(alignment_path):
    raise Exception(f"The camera alignment file '{alignment_path}' exists. If you want to re-compute the alignments and overwrite it, delete it first.")

# Load pointcloud processing envieronment
env = PointcloudEnv(
    camera_indices=[0, 1],
    camera_extrinsics_dir=extrinsics_dirname,
    # camera_alignment argument is used ONLY for multi-camera setting. Comment it out for single camera setting or when cameras are not aligned yet
    # camera_alignments="(filename)",
    camera_alignments=False # set to False when starting from sratch, when cameras are not aligned yet
)

# colors = [np.random.rand(3) for i in range(100)]
colors = [
    np.array([0.8, 0.2, 0.2]),
    np.array([0.2, 0.8, 0.2]),
    np.array([0.2, 0.2, 0.8])
    ]

# Visualize unaligned point clouds
'''
pcd = o3d.geometry.PointCloud()
for cam_id in env.camera_indices:
    processed_pcd = env.get_single_pcd(cam_id)

    # can crop it according to the specific task
    # processed_pcd = crop(processed_pcd, 0.2, 0.9, -0.5, 0.5, -0.12, 0.5)

    processed_pcd = processed_pcd
        
    processed_pcd.paint_uniform_color(colors[cam_id])
    pcd += processed_pcd

o3d.visualization.draw_geometries([pcd])
'''

# A dictionary of pointclouds collected from each camera
pcds = {}
for cam_id in [0, 1]:
    curr_pcd = env.get_single_pcd(cam_id)
    # We only focus on a small space on the table
    curr_pcd = crop(curr_pcd, 0.3, 0.7, -0.25, 0.2, -0.12, 0.5)
    pcds[cam_id] = curr_pcd

# Align all other cameras to camera 0. change this accordingly. 
base_cam_id = 0

# Align the point clouds
transforms = compute_align_to_target(
    # TODO: tune the threshold
    pcds[base_cam_id], pcds, threshold=0.5, visualize=False) # Set to True to visualize pairwise alignment

# Transform all point clouds
camera_aligned_pcds = align_pcds(pcds, transforms, [0, 1], colors = colors)
o3d.visualization.draw_geometries([camera_aligned_pcds])

# Remove color
camera_aligned_pcds_ = o3d.geometry.PointCloud(camera_aligned_pcds.points)
camera_aligned_pcds_.normals = camera_aligned_pcds.normals 
o3d.visualization.draw_geometries([camera_aligned_pcds_])

# Save alginment transformation matrix
# Save the transform matrices as npz
save_path = os.path.join(alignment_dirname, "camera_alignments_matrices.npz")
save_content = {
    str(cam_id): transform for cam_id, transform in transforms.items()
} # Convert cam_id to string to save as npz
np.savez(save_path, **save_content)

# Also save the humann readable transforms as xyz, quat into json
save_content = {}
for cam_id, transform in transforms.items():
    quat = Rotation.from_matrix(transform[:3, :3]).as_quat()
    save_content[cam_id] = {
        "xyz": transform[:3, 3].tolist(),
        "quaternion": quat.tolist()
    }

save_path = os.path.join(alignment_dirname, "camera_alignments_matrices.json")
with open(save_path, "w") as f:
    json.dump(save_content, f, indent=2)
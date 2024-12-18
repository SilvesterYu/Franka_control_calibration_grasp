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

alignment_dirname = os.path.join(parent_dir, "camera_alignment/alignment_results")
alignment_path = os.path.join(alignment_dirname, "camera_alignments_matrices.npz")

# TODO: Uncomment this if you are done with camera alignment and do not wish to accidentally to overwrite the alignment results
# if os.path.exists(alignment_path):
#     raise Exception(f"The camera alignment file '{alignment_path}' exists. If you want to re-compute the alignments and overwrite it, delete it first.")

from camera_alignment.pointcloud_env import PointcloudEnv
from camera_alignment.utils import *

# Load pointcloud processing envieronment
env = PointcloudEnv(
    camera_indices=[0, 1],
    camera_extrinsics_dir=extrinsics_dirname,
    # camera_alignment argument is used ONLY for multi-camera setting. Comment it out for single camera setting or when cameras are not aligned yet
    camera_alignments=alignment_path,
    # camera_alignments=False # set to False when starting from sratch, when cameras are not aligned yet
)

# colors = [np.random.rand(3) for i in range(100)]
colors = [
    np.array([0.8, 0.2, 0.2]),
    np.array([0.2, 0.8, 0.2]),
    np.array([0.2, 0.2, 0.8])
    ]

# Visualize unaligned point clouds

pcd = o3d.geometry.PointCloud()
for cam_id in env.camera_indices:
    processed_pcd = env.get_single_pcd(cam_id)

    # can crop it according to the specific task
    processed_pcd = crop(processed_pcd, 0.2, 0.9, -0.5, 0.5, -0.12, 0.5)

    processed_pcd = processed_pcd
        
    processed_pcd.paint_uniform_color(colors[cam_id])
    pcd += processed_pcd

o3d.visualization.draw_geometries([pcd])

#### You can save the pcd here (Note from Lifan)

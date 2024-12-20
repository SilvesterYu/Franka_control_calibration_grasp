import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

objects = np.load("objects.npy")
transforms = np.load("transforms.npy")
initial_pcd = np.load("initial_pcd.npy")
initial_pcd_seg = np.load("initial_pcd_seg.npy")

print(objects)
print(transforms[0])

def plot_pcd(pts3d, pcd_seg = None):

    pcd = np.zeros_like(pts3d, dtype = np.float64)
    pcd[:, :] = pts3d[:, :]
    
    pts_vis = o3d.geometry.PointCloud()
    pts_vis.points = o3d.utility.Vector3dVector(pcd)
    
    if pcd_seg is not None:    

        seg_ids = np.unique(pcd_seg)
        n = len(seg_ids)
        cmap = plt.get_cmap("tab10")
        id_to_color = {uid: cmap(i / n)[:3] for i, uid in enumerate(seg_ids)}
        colors = np.array([id_to_color[seg_id] for seg_id in pcd_seg])
        # print("Seg IDs = ", seg_ids)
        # print("Colors = ", id_to_color)
        pts_vis.colors = o3d.utility.Vector3dVector(colors)

    o3d.visualization.draw_geometries([pts_vis])


plot_pcd(initial_pcd, initial_pcd_seg)
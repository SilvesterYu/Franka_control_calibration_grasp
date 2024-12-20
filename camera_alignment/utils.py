import numpy as np
import open3d as o3d
import numpy as np
import copy
import os
import sys
def display_inlier_outlier(cloud, ind):
    # Compute normals to help visualize
    # cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    inlier_cloud = cloud.select_by_index(ind)
    inlier_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    outlier_cloud = cloud.select_by_index(ind, invert=True)
    print("Showing outliers (red) and inliers (gray): ")
    outlier_cloud.paint_uniform_color([1, 0, 0])
    inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
def preprocess_point_cloud(pcd, voxel_size):
    print(":: Random downsample to 1/4 size.")
    pcd_size = len(pcd.points)
    pcd_down_mask = np.random.choice(pcd_size, int(pcd_size / 4))
    pcd_down = pcd.select_by_index(pcd_down_mask)
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd_down.voxel_down_sample(voxel_size)
    # o3d.visualization.draw_geometries([pcd_down])
    radius = 0.02
    print(":: Remove radius outlier with radius %.3f." % radius)
    nb_points = int(((radius / voxel_size) ** 2) * 0.9) 
    # cl, ind = pcd_down.remove_radius_outlier(nb_points=nb_points, radius=radius)
    cl, ind = pcd_down.remove_statistical_outlier(nb_neighbors=32, std_ratio=0.3)
    display_inlier_outlier(pcd_down, ind)
    pcd_down = cl
    radius_normal = voxel_size * 4
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh
# To crop pointcloud
def crop(pcd, x0, x1, y0, y1, z0, z1):
    points = np.asarray(pcd.points)
    pcd_sel = pcd.select_by_index(np.where(points[:, 0] > x0)[0])
    points = np.asarray(pcd_sel.points)
    pcd_sel = pcd_sel.select_by_index(np.where(points[:, 0] < x1)[0])
    points = np.asarray(pcd_sel.points)
    pcd_sel = pcd_sel.select_by_index(np.where(points[:, 1] > y0)[0])
    points = np.asarray(pcd_sel.points)
    pcd_sel = pcd_sel.select_by_index(np.where(points[:, 1] < y1)[0])
    points = np.asarray(pcd_sel.points)
    pcd_sel = pcd_sel.select_by_index(np.where(points[:, 2] > z0)[0])
    points = np.asarray(pcd_sel.points)
    pcd_sel = pcd_sel.select_by_index(np.where(points[:, 2] < z1)[0])
    return pcd_sel
#################################### Multi Camera Alignment ####################################
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])
def compute_align_to_target(target_pcd, other_pcds, base_cam_id = 0, threshold=0.01, visualize=False):
    """
    Compute alignments from other_pcds to base_pcd
    
    Input:
        target_pcd: Open3D point cloud
        other_pcds: dict of Open3D point clouds {cam_id: pcd}
    Return:
        dict of transforms {cam_id: transforms}
    """
    transforms = {}
    for cam_id, source in other_pcds.items():        
        print(f":: Aligning camera {cam_id} with target pcd")
        print(":: Apply point-to-point ICP")
        trans_init = np.identity(4)
        # reg_p2p = o3d.pipelines.registration.registration_icp(
        #     source, target, threshold, trans_init,
        #     o3d.pipelines.registration.TransformationEstimationPointToPoint())
        source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        target_pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
        reg_p2p = o3d.pipelines.registration.registration_icp(
            source, target_pcd, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPlane())
        print(reg_p2p)
        print("Transformation is:")
        print(reg_p2p.transformation)
        if visualize:
            draw_registration_result(source, target_pcd, reg_p2p.transformation)
        transforms[cam_id] = reg_p2p.transformation.copy()
    
    # For base camer, set to identity
    transforms[base_cam_id] = np.identity(4)
    print("Transforms is:")
    print(reg_p2p.transformation)
    
    return transforms
def align_pcds(pcds, transforms, ind = [0, 1], 
               # R, G, B by default, change this accordingly
               colors = [
                        np.array([0.8, 0.2, 0.2]),
                        np.array([0.2, 0.8, 0.2]),
                        np.array([0.2, 0.2, 0.8])
                        ]):
    """
    Align point clouds using transforms
    
    Input:
        pcds: dict of Open3D point clouds {cam_id: pcd}
        transforms: dict of transforms {cam_id: transforms}.
    Return:
        Open3D point cloud
    """
    transformed_pcds = o3d.geometry.PointCloud()
    for cam_id in ind:
        transformed_pcd = pcds[cam_id].transform(transforms[cam_id])
        transformed_pcd.paint_uniform_color(colors[cam_id])
        transformed_pcds += transformed_pcd
    
    return transformed_pcds
# usage
if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    parent_dir = os.path.dirname(current_dir)
    sys.path.append(parent_dir)
    extrinsics_dirname = os.path.join(parent_dir, "camera_calibration", "calibration_results")
    from pointcloud_env import PointcloudEnv
    env = PointcloudEnv(
        camera_indices=[0],
        camera_extrinsics_dir=extrinsics_dirname,
        # camera_alignments="(filename)",
        camera_alignments=False # set to False to start from scratch
    )
    # colors = [np.random.rand(3) for i in range(100)]
    colors = [
        np.array([0.8, 0.2, 0.2]),
        np.array([0.2, 0.8, 0.2]),
        np.array([0.2, 0.2, 0.8])
        ]
    
    voxel_size = 0.005
    # Obtain the point cloud
    pcd = env.get_single_raw_pcd(0)
    pcd = o3d.geometry.PointCloud(pcd)
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    o3d.visualization.draw_geometries([pcd])
    print(f"PCD shape: {len(pcd.points)}")
    pcd_down, pcd_fpfh = preprocess_point_cloud(pcd, voxel_size)
    o3d.visualization.draw_geometries([pcd_down])
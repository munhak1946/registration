# Open3D: www.open3d.org
# The MIT License (MIT)
# See license file or visit www.open3d.org for details

# examples/Python/Tutorial/Advanced/multiway_registration.py

from open3d import *
import numpy as np

voxel_size = 0.02
max_correspondence_distance_coarse = voxel_size * 15
max_correspondence_distance_fine = voxel_size * 1.5


def load_point_clouds(voxel_size = 0.0):
    pcds = []
    for i in range(8):
        pcd = read_point_cloud("C:\Users\sujin\PycharmProjects\untitled2\TestData\F%d.ply" % i)
        pcd_down = voxel_down_sample(pcd, voxel_size = voxel_size)
        pcds.append(pcd_down)
    return pcds


def pairwise_registration(source, target):
    print("Apply point-to-plane ICP")
    icp_coarse = registration_icp(source, target,
            max_correspondence_distance_coarse, np.identity(4),
            TransformationEstimationPointToPlane())
    icp_fine = registration_icp(source, target,
            max_correspondence_distance_fine, icp_coarse.transformation,
            TransformationEstimationPointToPlane())
    transformation_icp = icp_fine.transformation
    information_icp = get_information_matrix_from_point_clouds(
            source, target, max_correspondence_distance_fine,
            icp_fine.transformation)
    return transformation_icp, information_icp


def full_registration(pcds,
        max_correspondence_distance_coarse, max_correspondence_distance_fine):
    pose_graph = PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(PoseGraphNode(odometry))
    n_pcds = len(pcds)
    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            transformation_icp, information_icp = pairwise_registration(
                    pcds[source_id], pcds[target_id])
            print("Build PoseGraph")
            if target_id == source_id + 1: # odometry case
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(PoseGraphNode(np.linalg.inv(odometry)))
                pose_graph.edges.append(PoseGraphEdge(source_id, target_id,
                        transformation_icp, information_icp, uncertain = False))
            else: # loop closure case
                pose_graph.edges.append(PoseGraphEdge(source_id, target_id,
                        transformation_icp, information_icp, uncertain = True))
    return pose_graph


if __name__ == "__main__":

    set_verbosity_level(VerbosityLevel.Debug)
    pcds_down = load_point_clouds(voxel_size)
    draw_geometries(pcds_down)

    print("Full registration ...")
    pose_graph = full_registration(pcds_down,
            max_correspondence_distance_coarse,
            max_correspondence_distance_fine)

    print("Optimizing PoseGraph ...")
    option = GlobalOptimizationOption(
            max_correspondence_distance = max_correspondence_distance_fine,
            edge_prune_threshold = 0.75,
            reference_node = 0)
    global_optimization(pose_graph,
            GlobalOptimizationLevenbergMarquardt(),
            GlobalOptimizationConvergenceCriteria(), option)

    print("Transform points and display")
    for point_id in range(len(pcds_down)):
        print(pose_graph.nodes[point_id].pose)
        pcds_down[point_id].transform(pose_graph.nodes[point_id].pose)
    draw_geometries(pcds_down)

    print("Make a combined point cloud")
    pcds = load_point_clouds(voxel_size)
    pcd_combined = PointCloud()
    for point_id in range(len(pcds)):
        pcds[point_id].transform(pose_graph.nodes[point_id].pose)
        pcd_combined += pcds[point_id]
    pcd_combined_down = voxel_down_sample(pcd_combined, voxel_size = voxel_size)
    write_point_cloud("multiway_registration.pcd", pcd_combined_down)
    draw_geometries([pcd_combined_down])

from open3d import *
import numpy as np
import copy
import os

index = 0

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    draw_geometries([source_temp, target_temp])

def draw_registration_result_original_color(source, target, transformation):
    source_temp = copy.deepcopy(source)
    source_temp.transform(transformation)
    draw_geometries([source_temp, target])

def save_ply(source,target,transformation,index):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    source_temp += target_temp
    temp = PointCloud()
    temp = source_temp
    writer_name = "writer9-" + str(index)
    write_point_cloud("C:\Users\sujin\PycharmProjects\untitled2\TestData\Test/" + writer_name + ".ply", temp, True);

def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = voxel_down_sample(pcd, voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    estimate_normals(pcd_down, KDTreeSearchParamHybrid(
            radius = radius_normal, max_nn = 30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = compute_fpfh_feature(pcd_down, KDTreeSearchParamHybrid(radius = radius_feature, max_nn = 100))
    return pcd_down, pcd_fpfh
def execute_global_registration(
        source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = registration_ransac_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            distance_threshold,
            TransformationEstimationPointToPoint(False), 4,
            [CorrespondenceCheckerBasedOnEdgeLength(0.9),
            CorrespondenceCheckerBasedOnDistance(distance_threshold)],
            RANSACConvergenceCriteria(4000000, 500))
    return result
def execute_fast_global_registration(source_down, target_down,
        source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.5
    print(":: Apply fast global registration with distance threshold %.3f" \
            % distance_threshold)
    result = registration_fast_based_on_feature_matching(
            source_down, target_down, source_fpfh, target_fpfh,
            FastGlobalRegistrationOption(
            maximum_correspondence_distance = distance_threshold))
    return result

def refine_registration(source, target, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: Point-to-plane ICP registration is applied on original point")
    print("   clouds to refine the alignment. This time we use a strict")
    print("   distance threshold %.3f." % distance_threshold)
    result = registration_icp(source, target, distance_threshold,
            result_ransac.transformation,
            TransformationEstimationPointToPlane())
    return result

def data_load(dir):
    fileList = os.listdir(dir)

    source = read_point_cloud(dir + fileList[index])
    if index>=len(fileList)-1:
        target = read_point_cloud(dir + fileList[14])
    else:
        target = read_point_cloud(dir + fileList[index + 1])
    return source,target

if __name__ == "__main__":
    voxel_size = 0.05 # means 5cm for the dataset
    # C:\Users\sujin\PycharmProjects\untitled2\TestData
    dir = 'C:\Users\sujin\PycharmProjects\untitled2\TestData\Test/'

    # dir=os.path.abspath(dir)
    print(os.getcwd())
    # print(os.path.abspath(dir))
    fileList = os.listdir(dir)
    print(fileList)
    while(index<len(fileList)*2-2):

        if index == 14:
            index += 1
        source,target = data_load(dir)

        trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0],
                                 [1.0, 0.0, 0.0, 0.0],
                                 [0.0, 1.0, 0.0, 0.0],
                                 [0.0, 0.0, 0.0, 1.0]])
        source.transform(trans_init)

        source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
        target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
        index += 2

        # Under the line is Registration & Save
        result_ransac = execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)
        draw_registration_result(source, target, result_ransac.transformation) #ICP for PointToPoint Registration

        result_icp = refine_registration(source, target,source_fpfh, target_fpfh, voxel_size)
        draw_registration_result(source, target, result_icp.transformation) # ICP for PointToPlane Registration
        save_ply(source,target,result_icp.transformation,index/2)


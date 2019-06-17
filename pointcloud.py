# Open3D: www.open3d.org
# The MIT License (MIT)
# See license file or visit www.open3d.org for details

# examples/Python/Tutorial/Basic/pointcloud.py

import numpy as np
from open3d import *

if __name__ == "__main__":

    print("Load a ply point cloud, print it, and render it")

    downpcd = read_point_cloud("C:\Users\sujin\PycharmProjects\untitled2\TestData\Test\Dec_Filter_3.ply")
    # print(pcd)
    # print(np.asarray(pcd.points))
    # draw_geometries([downpcd])

    # print("Downsample the point cloud with a voxel of 0.01")

    # downpcd = voxel_down_sample(pcd, voxel_size = 0.01)
    # draw_geometries([downpcd])

    # write_point_cloud("../../TestData/copy_of_test12_2_1.ply", downpcd)
    # print("Recompute the normal of the downsampled point cloud")


    estimate_normals(downpcd, search_param = KDTreeSearchParamHybrid(radius = 0.1, max_nn = 30))


    # draw_geometries([downpcd])


    write_point_cloud("C:\Users\sujin\PycharmProjects\untitled2\TestData\Test\Dec_Filter_voxel_3.ply", downpcd)


    # print("Print a normal vector of the 0th point")
    # print(downpcd.normals[0])
    # print("Print the normal vectors of the first 10 points")
    # print(np.asarray(downpcd.normals)[:10,:])
    # print("")

    # print("Load a polygon volume and use it to crop the original point cloud")
    # vol = read_selection_polygon_volume("../../TestData/Crop/cropped.json")
    # chair = vol.crop_point_cloud(pcd)
    # draw_geometries([chair])
    # print("")

    # print("Paint chair")
    # chair.paint_uniform_color([1, 0.706, 0])
    # draw_geometries([chair])
    # print("")

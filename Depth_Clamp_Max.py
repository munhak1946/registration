import pyrealsense2 as rs
import time


device = rs.context().query_devices()[0]
advnc_mode = rs.rs400_advanced_mode(device)
depth_table_control_group = advnc_mode.get_depth_table()

#여기서 단위는 mm입니다.
depth_table_control_group.depthClampMin = 0 #최소 카메라로부터 x_mm 떨어진곳부터 저장이 됩니다.
depth_table_control_group.depthClampMax = 1000 #여기서 변수를 바꿔줌에 따라서 얼마까지의 거리를 잡아줄지 설정 할 수 있습니다. 최대 이 거리 까지 포인트로 저장됩니다.
advnc_mode.set_depth_table(depth_table_control_group)

# Declare pointcloud object, for calculating pointclouds and texture mappings
pc = rs.pointcloud()
# We want the points object to be persistent so we can display the last cloud when a frame drops
points = rs.points()

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()
#Start streaming with default recommended configuration
pipe.start();
index = 0
try:
    while index < 10: #added this
        time.sleep(1)
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()
        # Fetch color and depth frames
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        
        #필터 적용 코드
        #dec_filter = rs.decimation_filter()  # Decimation - reduces depth frame density
        #spat_filter = rs.spatial_filter()  # Spatial    - edge-preserving spatial smoothing
        #temp_filter = rs.temporal_filter()  # Temporal   - reduces temporal noise
        #hole_filling = rs.hole_filling_filter()  # hole filling - 빈곳을 채워줌
        #depth = dec_filter.process(depth);
        #depth = spat_filter.process(depth);
        #depth = temp_filter.process(depth);
        #depth = hole_filling.process(depth);

        # Tell pointcloud object to map to this color frame
        pc.map_to(color)

        # Generate the pointcloud and texture mappings
        points = pc.calculate(depth)

        print("Saving to %d.ply..." %index)
        points.export_to_ply("%d.ply" %index, color)
        print("Done")
        index += 1

finally:
    pipe.stop()
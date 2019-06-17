import time
import pyrealsense2 as rs

device = rs.context().query_devices()[0]
depth_seonsor = device.query_sensors()[0]
laser_pwr = depth_seonsor.get_option(rs.option.laser_power)
print("laser poser = ",laser_pwr)
laser_range = depth_seonsor.get_option_range(rs.option.laser_power)
set_laser = 120
depth_seonsor.set_option(rs.option.laser_power,set_laser)

advnc_mode = rs.rs400_advanced_mode(device)
depth_table_control_group = advnc_mode.get_depth_table()

depth_table_control_group.depthClampMin = 0
depth_table_control_group.depthClampMax = 1500
advnc_mode.set_depth_table(depth_table_control_group)

# Declare pointcloud object, for calculating pointclouds and texture mappings
pc = rs.pointcloud()
# We want the points object to be persistent so we can display the last cloud when a frame drops
points = rs.points()

pipe = rs.pipeline()
# Start streaming with default recommended configuration
pipe.start();
index = 0
try:
    while True:
        frames = pipe.wait_for_frames()
        # Fetch color and depth frames
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()

        #filter definition
        dec_filter = rs.decimation_filter() # Decimation - reduces depth frame density
        spat_filter = rs.spatial_filter() # Spatial - edge-preserving spatial smoothing
        temp_filter = rs.temporal_filter() # Temporal - reduces temporal noise

        depth_to_disparity = rs.disparity_transform(True)
        disparity_to_depth = rs.disparity_transform(False)

        #Using Filtering
        frame = dec_filter.process(depth)
        spat_filter.set_option(rs.option.filter_magnitude, 4)
        spat_filter.set_option(rs.option.holes_fill, 3)
        frame = depth_to_disparity.process(frame)
        frame = spat_filter.process(frame)
        # frame = dec_filter.process(frame)
        frame = temp_filter.process(frame)
        frame = disparity_to_depth.process(frame)

        pc.map_to(color) # Tell pointcloud object to map to this color frame
        points = pc.calculate(frame) # Generate the pointcloud and texture mappings

        points.export_to_ply("./option_data/%d.ply" % index, color)
        print("Done")
        time.sleep(5)
finally:
    pipe.stop()
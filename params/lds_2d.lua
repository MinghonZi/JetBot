-- https://google-cartographer-ros.readthedocs.io/en/latest/configuration.html
-- https://github.com/ROBOTIS-GIT/turtlebot3/blob/master/turtlebot3_slam/config/turtlebot3_lds_2d.lua
-- https://github.com/jackal/jackal_cartographer_navigation/blob/melodic-devel/config/jackal.lua
-- ros2 bag record --all
-- ros2 run cartographer_ros cartographer_rosbag_validate -bag_filename <rosbag2_dir>

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
  -- https://github.com/cartographer-project/cartographer/issues/1569#issuecomment-491468305
  imu_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 -- LDS-01 outputs a single message per revolution.
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options

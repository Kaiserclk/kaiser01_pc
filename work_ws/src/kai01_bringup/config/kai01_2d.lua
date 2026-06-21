-- Cartographer 2D SLAM configuration for kai01 robot

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- TF frame configuration
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,

  -- Use odometry from the robot
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,

  -- Laser scan configuration
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- Timing
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  -- Sampling ratios for rates
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- MAP_BUILDER parameters
MAP_BUILDER.use_trajectory_builder_2d = true

-- TRAJECTORY_BUILDER_2D parameters
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 16.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- Ceres scan matcher
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- Motion filter
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 5.0
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.2
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.)

-- Submaps
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 100
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type = "PROBABILITY_GRID"
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Pose graph (loop closure)
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimize_every_n_nodes = 90
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.global_sampling_ratio = 0.003
POSE_GRAPH.global_constraint_search_after_n_seconds = 10.
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.ceres_scan_matcher.occupied_space_weight = 20.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher.translation_weight = 10.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher.rotation_weight = 1.0

return options

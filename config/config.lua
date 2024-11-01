include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.05,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D = {
  use_imu_data = true,
  min_range = 0.1,
  max_range = 10.0,
  missing_data_ray_length = 1.0,
  num_accumulated_range_data = 1,
  voxel_filter_size = 0.025,
  use_online_correlative_scan_matcher = false,
  ceres_scan_matcher = {
    occupied_space_weight = 1,
    translation_weight = 10,
    rotation_weight = 40,
    ceres_solver_options = {
      use_nonmonotonic_steps = false,
      max_num_iterations = 20,
      num_threads = 1,
    },
  },
  motion_filter = {
    max_time_seconds = 0.1,
    max_distance_meters = 0.1,
    max_angle_radians = math.rad(0.1),
  },
  submaps = {
    num_range_data = 90,
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",
      resolution = 0.05,
    },
    range_data_inserter = {
      insert_free_space = true,
      hit_probability = 0.55,
      miss_probability = 0.49,
    },
  },

  POSE_GRAPH = {
    optimize_every_n_nodes = 90,
    constraint_builder = {
      min_score = 0.65,
      global_localization_min_score = 0.7,
      sampling_ratio = 0.3,
      max_constraint_distance = 15.0,
      adaptive_voxel_filter = {
        max_length = 0.5,
        min_num_points = 100,
        max_range = 50.0,
      },
      ceres_scan_matcher = {
        occupied_space_weight = 1.0,
        translation_weight = 10.0,
        rotation_weight = 1.0,
      },
    },
    optimization_problem = {
      huber_scale = 1e1,
      acceleration_weight = 1e-1,
      rotation_weight = 1e-1,
      local_slam_pose_translation_weight = 1e5,
      local_slam_pose_rotation_weight = 1e5,
      odometry_translation_weight = 1e5,
      odometry_rotation_weight = 1e5,
    },
    max_num_final_iterations = 200,
  }
}

TRAJECTORY_BUILDER_2D.use_odometry = true
TRAJECTORY_BUILDER_2D.odometry_topic = "/odometry/filtered"

return options
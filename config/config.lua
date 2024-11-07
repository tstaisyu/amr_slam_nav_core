-- Cartographer configuration for a 2D LIDAR-based SLAM
include "map_builder.lua"
include "trajectory_builder.lua"

-- Global SLAM and local SLAM configuration options
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",  -- Global map frame
  tracking_frame = "base_link",  -- Frame for localizing the robot
  published_frame = "base_link",  -- Frame for published poses
  odom_frame = "odom",  -- Frame attached to odometry information
  provide_odom_frame = false,  -- Whether to output odometry data
  publish_frame_projected_to_2d = true,  -- Force projection of 3D to 2D
  use_odometry = true,  -- Utilize odometry data if available
  use_nav_sat = false,  -- Use GPS data (if available)
  use_landmarks = false,  -- Use landmark data (if available)
  num_laser_scans = 1,  -- Number of single laser scans to use
  num_multi_echo_laser_scans = 0,  -- Number of multi-echo laser scans
  num_subdivisions_per_laser_scan = 1,  -- Subdivision of scans for processing
  num_point_clouds = 0,  -- Number of point clouds
  lookup_transform_timeout_sec = 0.2,  -- Timeout for transform lookups
  submap_publish_period_sec = 0.3,  -- Publishing period for submaps
  pose_publish_period_sec = 0.05,  -- Publishing period for poses
  trajectory_publish_period_sec = 30e-3,  -- Publishing period for trajectory updates
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- Enabling 2D SLAM components
MAP_BUILDER.use_trajectory_builder_2d = true

-- Configuration of the 2D trajectory builder
TRAJECTORY_BUILDER_2D = {
  use_imu_data = true,  -- Utilize IMU data for motion correction
  min_range = 0.1,
  max_range = 10.0,
  missing_data_ray_length = 1.0,
  num_accumulated_range_data = 1,
  voxel_filter_size = 0.025,  -- Voxel size for downsampling point clouds
  use_online_correlative_scan_matcher = false,  -- Online scan matching
  ceres_scan_matcher = {
    occupied_space_weight = 1,  -- Weight for occupied space in Ceres
    translation_weight = 10,  -- Weight for translation in Ceres optimization
    rotation_weight = 40,  -- Weight for rotation in Ceres optimization
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
    num_range_data = 90,  -- Number of scans to create a submap
    grid_options_2d = {
      grid_type = "PROBABILITY_GRID",
      resolution = 0.05,
    },
    range_data_inserter = {
      insert_free_space = true,  -- Inserting free space into submaps
      hit_probability = 0.55,  -- Probability for laser hits
      miss_probability = 0.49,  -- Probability for misses
    },
  },

  POSE_GRAPH = {
    optimize_every_n_nodes = 90,  -- Optimization frequency for pose graph
    constraint_builder = {
      min_score = 0.65,  -- Minimum score for adding constraints
      global_localization_min_score = 0.7,  -- Score for global localization
      sampling_ratio = 0.3,  -- Sampling ratio for constraints
      max_constraint_distance = 15.0,  -- Maximum distance for constraints
      adaptive_voxel_filter = {
        max_length = 0.5,  -- Maximum voxel length
        min_num_points = 100,  -- Minimum number of points in a voxel
        max_range = 50.0,  -- Maximum range of the sensor
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
    max_num_final_iterations = 200,  -- Maximum iterations for final optimization
  }
}

-- Set odometry topic
TRAJECTORY_BUILDER_2D.use_odometry = true
TRAJECTORY_BUILDER_2D.odometry_topic = "/odometry/filtered"

-- Return configuration options
return options
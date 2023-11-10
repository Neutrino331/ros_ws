include "map_builder.lua"  
include "trajectory_builder.lua"  

options = {
  map_builder = MAP_BUILDER,  
  trajectory_builder = TRAJECTORY_BUILDER,  
  map_frame = "map", -- 地圖框的名稱
  tracking_frame = "laser_frame", -- 追蹤訊框的名稱
  published_frame = "odom", -- 發布幀的名稱
  odom_frame = "odom", -- 里程計幀的名稱
  provide_odom_frame = false, -- 是否提供里程計幀
  publish_frame_projected_to_2d = true, -- 是否發布2d姿態
  use_pose_extrapolator = true,
  use_odometry = true, -- 是否使用里程計
  use_nav_sat = false, -- 是否使用導航衛星
  use_landmarks = false, -- 是否使用地標
  num_laser_scans = 1, -- 雷射雷達的數量
  num_multi_echo_laser_scans = 0, -- 多回波雷射雷達的數量
  num_subdivisions_per_laser_scan = 1, -- 每個雷射掃描的細分數量
  num_point_clouds = 0, -- 點雲的數量
  lookup_transform_timeout_sec = 0.2, -- 找出變換的逾時時間（秒）
  submap_publish_period_sec = 0.3, -- 子地圖發布週期（秒）
  pose_publish_period_sec = 5e-3, -- 姿態發布週期（秒）
  trajectory_publish_period_sec = 30e-3, -- 軌跡發布週期（秒）
  rangefinder_sampling_ratio = 1., -- 測距儀採樣比
  odometry_sampling_ratio = 1., -- 里程計採樣比
  fixed_frame_pose_sampling_ratio = 1., -- 固定幀姿態採樣比率
  imu_sampling_ratio = 1., -- IMU採樣比率
  landmarks_sampling_ratio = 1., -- 地標採樣比率
}
 
MAP_BUILDER.use_trajectory_builder_2d = true -- 是否啟動2D SLAM
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35 -- 2D軌跡建構器中子地圖的範圍資料數量
TRAJECTORY_BUILDER_2D.min_range = 0.1 -- 限制在雷達最小掃描範圍，比機器人半徑小的都忽略
TRAJECTORY_BUILDER_2D.max_range = 3.5 -- 限制在雷達最大掃描範圍
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5. -- 限制在雷達最大掃描範圍
TRAJECTORY_BUILDER_2D.use_imu_data = false -- 是否使用IMU數據
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true -- 是否使用即時回環檢測掃描匹配

TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1) -- 1.0改成0.1,提高對運動的敏感度
POSE_GRAPH.constraint_builder.min_score = 0.65 -- 0.55改為0.65,Fast csm的最低分數，高於此分數才進行最佳化。
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7 --0.6改為0.7,全域定位最小分數，低於此分數則認為目前全域定位不準確

return options
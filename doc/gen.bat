@echo off & color 0A

:: protoc³ÌÐòÃû
set "PROTOC_EXE=protoc.exe"


set "PROTOC_PATH=%cd%"
set "CPP_OUT_PATH=%cd%"




set "PROTOC_FILE_NAME=./cartographer/common/proto/ceres_solver_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"

set "PROTOC_FILE_NAME=./cartographer/sensor/proto/adaptive_voxel_filter_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/sensor/proto/configuration.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/sensor/proto/sensor.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"

set "PROTOC_FILE_NAME=./cartographer/transform/proto/transform.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"


set "PROTOC_FILE_NAME=./cartographer/kalman_filter/proto/pose_tracker_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"

set "PROTOC_FILE_NAME=./cartographer/mapping/proto/map_builder_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping/proto/scan_matching_progress.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping/proto/sparse_pose_graph.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping/proto/sparse_pose_graph_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping/proto/submap_visualization.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping/proto/trajectory.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping/proto/trajectory_builder_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping/proto/trajectory_connectivity.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"

set "PROTOC_FILE_NAME=./cartographer/mapping/sparse_pose_graph/proto/constraint_builder_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping/sparse_pose_graph/proto/optimization_problem_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"

set "PROTOC_FILE_NAME=./cartographer/mapping_2d/proto/hybrid_grid.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping_2d/proto/motion_filter_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping_2d/proto/laser_fan_inserter_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping_2d/proto/cell_limits.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping_2d/proto/map_limits.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping_2d/proto/probability_grid.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping_2d/proto/range_data_inserter_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping_2d/proto/local_trajectory_builder_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping_2d/proto/submaps_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"

set "PROTOC_FILE_NAME=./cartographer/mapping_2d/scan_matching/proto/ceres_scan_matcher_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping_2d/scan_matching/proto/fast_correlative_scan_matcher_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping_2d/scan_matching/proto/real_time_correlative_scan_matcher_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"

set "PROTOC_FILE_NAME=./cartographer/mapping_3d/proto/hybrid_grid.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping_3d/proto/kalman_local_trajectory_builder_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping_3d/proto/laser_fan_inserter_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping_3d/proto/local_trajectory_builder_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping_3d/proto/motion_filter_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping_3d/proto/optimizing_local_trajectory_builder_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping_3d/proto/range_data_inserter_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping_3d/proto/submaps_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"

set "PROTOC_FILE_NAME=./cartographer/mapping_3d/scan_matching/proto/ceres_scan_matcher_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"
set "PROTOC_FILE_NAME=./cartographer/mapping_3d/scan_matching/proto/fast_correlative_scan_matcher_options.proto"
"%PROTOC_PATH%\%PROTOC_EXE%" --proto_path="%PROTOC_PATH%" --cpp_out="%CPP_OUT_PATH%" "%PROTOC_PATH%\%PROTOC_FILE_NAME%"


pause
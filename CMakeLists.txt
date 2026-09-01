cmake_minimum_required(VERSION 3.16)
project(rgbd_fullcalib)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  calibrate_mid360s_imu.sh
  tools/mid360s_imu_pipeline_node.py
  tools/mid360s_imu_runtime.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  tools/record_mid360s_imu_poses.py
  tools/calibrate_mid360s_imu_intrinsics.py
  tools/promote_mid360s_imu.py
  tools/imu_intrinsic.py
  DESTINATION lib/${PROJECT_NAME}/tools
)

# The promoter uses the same validator as the web viewer so deployment and UI
# cannot disagree about whether the formal result is usable.
install(FILES
  viewer/lidar_calib.py
  DESTINATION lib/${PROJECT_NAME}/viewer
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# Mechanical assets may be supplied independently.  Install either canonical
# location when present without making a source-only checkout fail to build.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/hardware")
  install(DIRECTORY hardware/
    DESTINATION share/${PROJECT_NAME}/hardware
  )
endif()
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/assets/rig")
  install(DIRECTORY assets/rig/
    DESTINATION share/${PROJECT_NAME}/assets/rig
  )
endif()

ament_package()

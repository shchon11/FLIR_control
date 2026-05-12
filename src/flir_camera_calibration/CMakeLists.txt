cmake_minimum_required(VERSION 3.8)
project(flir_camera_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if("$ENV{ROS_VERSION}" STREQUAL "1")
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
  )
  find_package(OpenCV REQUIRED COMPONENTS calib3d highgui imgcodecs imgproc)

  catkin_package(CATKIN_DEPENDS
    roscpp
    sensor_msgs
  )

  install(PROGRAMS
    scripts/noetic_intrinsic_calibration_node.py
    scripts/noetic_extrinsic_calibration_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(DIRECTORY launch config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
  install(FILES README.md
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
  return()
endif()

find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS calib3d highgui imgcodecs imgproc)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(flir_camera_calibration_node
  src/flir_camera_calibration_node.cpp
)

add_executable(flir_camera_extrinsic_calibration_node
  src/flir_camera_extrinsic_calibration_node.cpp
)

target_compile_features(flir_camera_calibration_node PUBLIC cxx_std_17)
target_compile_features(flir_camera_extrinsic_calibration_node PUBLIC cxx_std_17)

target_include_directories(flir_camera_calibration_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(flir_camera_extrinsic_calibration_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(flir_camera_calibration_node
  ${OpenCV_LIBS}
)

target_link_libraries(flir_camera_extrinsic_calibration_node
  ${OpenCV_LIBS}
)

ament_target_dependencies(flir_camera_calibration_node
  rclcpp
  sensor_msgs
)

ament_target_dependencies(flir_camera_extrinsic_calibration_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  flir_camera_calibration_node
  flir_camera_extrinsic_calibration_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

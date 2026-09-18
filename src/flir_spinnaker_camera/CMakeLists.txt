cmake_minimum_required(VERSION 3.8)
project(flir_spinnaker_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgcodecs imgproc)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Threads REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FlirMetadata.msg"
  DEPENDENCIES std_msgs
)

set(SPINNAKER_ROOT "$ENV{SPINNAKER_ROOT}" CACHE PATH "Spinnaker SDK root directory")

set(_spinnaker_include_hints)
set(_spinnaker_library_hints)
if(SPINNAKER_ROOT)
  list(APPEND _spinnaker_include_hints
    "${SPINNAKER_ROOT}"
    "${SPINNAKER_ROOT}/include"
  )
  list(APPEND _spinnaker_library_hints
    "${SPINNAKER_ROOT}"
    "${SPINNAKER_ROOT}/lib"
    "${SPINNAKER_ROOT}/lib64"
  )
endif()

# SPINNAKER_ROOT must win over CMAKE_PREFIX_PATH. ROS distributions can ship a
# vendored libSpinnaker (ros-humble-spinnaker-camera-driver installs 4.2 into
# /opt/ros/humble/lib), and find_library searches CMAKE_PREFIX_PATH before
# HINTS. Without this first pass the headers and the library come from
# different SDKs, and Spinnaker fails at runtime with
# "System instance has not been initialized [-1002]".
if(SPINNAKER_ROOT)
  find_path(SPINNAKER_INCLUDE_DIR
    NAMES Spinnaker.h
    HINTS ${_spinnaker_include_hints}
    PATH_SUFFIXES include
    NO_DEFAULT_PATH
  )

  find_library(SPINNAKER_LIBRARY
    NAMES Spinnaker
    HINTS ${_spinnaker_library_hints}
    PATH_SUFFIXES lib lib64
    NO_DEFAULT_PATH
  )
endif()

find_path(SPINNAKER_INCLUDE_DIR
  NAMES Spinnaker.h
  HINTS ${_spinnaker_include_hints}
  PATH_SUFFIXES include
)

find_library(SPINNAKER_LIBRARY
  NAMES Spinnaker
  HINTS ${_spinnaker_library_hints}
  PATH_SUFFIXES lib lib64
)

if(NOT SPINNAKER_INCLUDE_DIR OR NOT SPINNAKER_LIBRARY)
  message(FATAL_ERROR
    "Spinnaker SDK not found. Set SPINNAKER_ROOT or make the SDK discoverable via CMAKE_PREFIX_PATH."
  )
endif()

get_filename_component(SPINNAKER_LIBRARY_DIR ${SPINNAKER_LIBRARY} DIRECTORY)

add_executable(flir_spinnaker_camera_node
  src/flir_spinnaker_camera_node.cpp
)

add_executable(flir_camera_extrinsics_tf_node
  src/flir_camera_extrinsics_tf_node.cpp
)

add_executable(flir_multicam_inventory_tool
  src/flir_multicam_inventory_tool.cpp
)

# ---- GPU JPEG encoder (optional) ----
# image_rgb/compressed can be demosaiced (NPP) and JPEG-encoded (nvJPEG) on the
# GPU instead of the acquisition thread; see src/gpu_jpeg_encoder.hpp. Only
# CUDA *libraries* are needed (no nvcc): a CUDA toolkit, or NVIDIA's pip wheels
# unpacked without sudo into third_party/cuda12:
#   pip3 install --no-deps --target third_party/cuda12 \
#     nvidia-cuda-runtime-cu12 nvidia-nvjpeg-cu12 nvidia-npp-cu12 nvidia-cuda-nvcc-cu12
# (the nvcc wheel only for its crt/ headers, which cuda_runtime_api.h includes)
# Without them the node builds exactly as before (CPU path only).
set(FLIR_CUDA_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/cuda12"
  CACHE PATH "Directory holding nvidia/{cuda_runtime,nvjpeg,npp} (pip wheels)")
option(FLIR_WITH_GPU_JPEG "Build the nvJPEG/NPP encoder when the libraries are found" ON)
set(_cuda_hints
  "${FLIR_CUDA_ROOT}/nvidia/cuda_runtime"
  "${FLIR_CUDA_ROOT}/nvidia/nvjpeg"
  "${FLIR_CUDA_ROOT}/nvidia/npp"
  /usr/local/cuda
)
find_path(FLIR_CUDART_INCLUDE_DIR cuda_runtime_api.h HINTS ${_cuda_hints} PATH_SUFFIXES include)
find_path(FLIR_NVJPEG_INCLUDE_DIR nvjpeg.h HINTS ${_cuda_hints} PATH_SUFFIXES include)
find_path(FLIR_NPP_INCLUDE_DIR nppi_color_conversion.h HINTS ${_cuda_hints} PATH_SUFFIXES include)
find_path(FLIR_CUDA_CRT_INCLUDE_DIR crt/host_defines.h
  HINTS ${_cuda_hints} "${FLIR_CUDA_ROOT}/nvidia/cuda_nvcc" PATH_SUFFIXES include)
find_library(FLIR_CUDART_LIBRARY NAMES cudart libcudart.so.12
  HINTS ${_cuda_hints} PATH_SUFFIXES lib lib64)
find_library(FLIR_NVJPEG_LIBRARY NAMES nvjpeg libnvjpeg.so.12
  HINTS ${_cuda_hints} PATH_SUFFIXES lib lib64)
find_library(FLIR_NPPICC_LIBRARY NAMES nppicc libnppicc.so.12
  HINTS ${_cuda_hints} PATH_SUFFIXES lib lib64)
find_library(FLIR_NPPC_LIBRARY NAMES nppc libnppc.so.12
  HINTS ${_cuda_hints} PATH_SUFFIXES lib lib64)

set(_node_rpath ${SPINNAKER_LIBRARY_DIR})
if(FLIR_WITH_GPU_JPEG AND FLIR_CUDART_INCLUDE_DIR AND FLIR_NVJPEG_INCLUDE_DIR AND FLIR_NPP_INCLUDE_DIR
    AND FLIR_CUDA_CRT_INCLUDE_DIR AND FLIR_CUDART_LIBRARY AND FLIR_NVJPEG_LIBRARY AND FLIR_NPPICC_LIBRARY AND FLIR_NPPC_LIBRARY)
  target_sources(flir_spinnaker_camera_node PRIVATE src/gpu_jpeg_encoder.cpp)
  target_compile_definitions(flir_spinnaker_camera_node PRIVATE FLIR_HAVE_GPU_JPEG=1)
  target_include_directories(flir_spinnaker_camera_node SYSTEM PRIVATE
    ${FLIR_CUDART_INCLUDE_DIR} ${FLIR_CUDA_CRT_INCLUDE_DIR} ${FLIR_NVJPEG_INCLUDE_DIR} ${FLIR_NPP_INCLUDE_DIR})
  target_link_libraries(flir_spinnaker_camera_node
    ${FLIR_NVJPEG_LIBRARY} ${FLIR_NPPICC_LIBRARY} ${FLIR_NPPC_LIBRARY} ${FLIR_CUDART_LIBRARY})
  foreach(_lib ${FLIR_CUDART_LIBRARY} ${FLIR_NVJPEG_LIBRARY} ${FLIR_NPPICC_LIBRARY})
    get_filename_component(_dir ${_lib} DIRECTORY)
    list(APPEND _node_rpath ${_dir})
  endforeach()
  list(REMOVE_DUPLICATES _node_rpath)
  message(STATUS "GPU JPEG encoder: ON (${FLIR_NVJPEG_LIBRARY})")
else()
  message(STATUS "GPU JPEG encoder: OFF (CUDA runtime / nvJPEG / NPP not found; see FLIR_CUDA_ROOT)")
endif()

target_compile_features(flir_spinnaker_camera_node PUBLIC cxx_std_17)
target_compile_features(flir_camera_extrinsics_tf_node PUBLIC cxx_std_17)
target_compile_features(flir_multicam_inventory_tool PUBLIC cxx_std_17)

target_include_directories(flir_spinnaker_camera_node SYSTEM PRIVATE
  ${SPINNAKER_INCLUDE_DIR}
)

target_include_directories(flir_multicam_inventory_tool SYSTEM PRIVATE
  ${SPINNAKER_INCLUDE_DIR}
)

target_include_directories(flir_spinnaker_camera_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(flir_spinnaker_camera_node
  ${OpenCV_LIBS}
  ${SPINNAKER_LIBRARY}
  Threads::Threads
)

target_link_libraries(flir_multicam_inventory_tool
  ${SPINNAKER_LIBRARY}
)

# --disable-new-dtags emits DT_RPATH rather than DT_RUNPATH. Only DT_RPATH is
# searched ahead of LD_LIBRARY_PATH, and ROS's setup.bash puts
# /opt/ros/humble/lib (which may hold a different vendored libSpinnaker) on
# LD_LIBRARY_PATH. Without this, SPINNAKER_LIBRARY_DIR loses at runtime even
# when it wins at link time.
# Spinnaker stays first on the RPATH (see above); the CUDA library dirs follow.
set_target_properties(flir_spinnaker_camera_node PROPERTIES
  BUILD_RPATH "${_node_rpath}"
  INSTALL_RPATH "${_node_rpath}"
  LINK_FLAGS "-Wl,--disable-new-dtags"
)

set_target_properties(flir_multicam_inventory_tool PROPERTIES
  BUILD_RPATH ${SPINNAKER_LIBRARY_DIR}
  INSTALL_RPATH ${SPINNAKER_LIBRARY_DIR}
  LINK_FLAGS "-Wl,--disable-new-dtags"
)

ament_target_dependencies(flir_spinnaker_camera_node
  rclcpp
  sensor_msgs
  std_msgs
)

ament_target_dependencies(flir_camera_extrinsics_tf_node
  geometry_msgs
  rclcpp
  tf2_ros
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(flir_spinnaker_camera_node ${cpp_typesupport_target})

install(TARGETS
  flir_spinnaker_camera_node
  flir_camera_extrinsics_tf_node
  flir_multicam_inventory_tool
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

ament_export_dependencies(rosidl_default_runtime)

ament_package()

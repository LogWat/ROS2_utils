cmake_minimum_required(VERSION 3.8)
project(utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# add_executable(livox_cloud_merger src/livox_cloud_merger.cpp)
add_library(livox_cloud_merger SHARED src/livox_cloud_merger.cpp)
add_library(vel2odom SHARED src/vel2odom.cpp)
add_library(livox_imu_collector SHARED src/livox_imu_collector.cpp)

rclcpp_components_register_nodes(livox_cloud_merger "utils::LivoxCloudMerger")
rclcpp_components_register_nodes(vel2odom "utils::Vel2Odom")
rclcpp_components_register_nodes(livox_imu_collector "utils::LivoxImuCollector")

target_compile_definitions(livox_cloud_merger
  PRIVATE "LIVOX_CLOUD_MERGER_BUILDING_LIBRARY"
)
target_compile_definitions(vel2odom
  PRIVATE "VEL2ODOM_BUILDING_LIBRARY"
)
target_compile_definitions(livox_imu_collector
  PRIVATE "LIVOX_IMU_COLLECTOR_BUILDING_LIBRARY"
)

target_include_directories(livox_cloud_merger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(vel2odom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(livox_imu_collector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(livox_cloud_merger PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(vel2odom PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(livox_imu_collector PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  livox_cloud_merger
  "rclcpp"
  "sensor_msgs"
  "pcl_ros"
  "rclcpp_components"
)

ament_target_dependencies(
  vel2odom
  "rclcpp"
  "sensor_msgs"
  "nav_msgs"
  "pcl_ros"
  "rclcpp_components"
)

ament_target_dependencies(
  livox_imu_collector
  "rclcpp"
  "sensor_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "rclcpp_components"
)

install(TARGETS livox_cloud_merger
  DESTINATION lib)
install(TARGETS vel2odom
  DESTINATION lib)
install (TARGETS livox_imu_collector
  DESTINATION lib)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

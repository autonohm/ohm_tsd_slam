cmake_minimum_required(VERSION 3.8)
project(ohm_tsd_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2 -std=c++1z)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Boost COMPONENTS thread REQUIRED)
find_package(OpenMP REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StartStopSLAM.srv"
)

add_executable(slam_node
  src/slam.cpp
  src/SlamNode.cpp
  src/ThreadLocalize.cpp
  src/ThreadSLAM.cpp
  src/ThreadMapping.cpp
  src/ThreadGrid.cpp
  src/OdometryAnalyzer.cpp
  src/obcore/base/Logger.cpp
  src/obcore/base/Time.cpp
  src/obcore/base/Timer.cpp
  src/obcore/base/tools.cpp
  src/obcore/math/linalg/gsl/Matrix.cpp
  src/obcore/math/linalg/gsl/Vector.cpp
  src/obcore/math/linalg/MatrixFactory.cpp
  src/obvision/reconstruct/Sensor.cpp
  src/obvision/reconstruct/grid/SensorPolar2D.cpp
  src/obvision/reconstruct/grid/TsdGrid.cpp
  src/obvision/reconstruct/grid/TsdGridBranch.cpp
  src/obvision/reconstruct/grid/TsdGridComponent.cpp
  src/obvision/reconstruct/grid/TsdGridPartition.cpp
  src/obvision/reconstruct/grid/RayCastAxisAligned2D.cpp
  src/obvision/reconstruct/grid/RayCastPolar2D.cpp
  src/obvision/registration/Trace.cpp
  src/obvision/registration/icp/ClosedFormEstimator2D.cpp
  src/obvision/registration/icp/Icp.cpp
  src/obvision/registration/icp/IcpMultiInitIterator.cpp
  src/obvision/registration/icp/PointToLineEstimator2D.cpp
  src/obvision/registration/icp/assign/AnnPairAssignment.cpp
  src/obvision/registration/icp/assign/FlannPairAssignment.cpp
  src/obvision/registration/icp/assign/PairAssignment.cpp
  src/obvision/registration/icp/assign/ProjectivePairAssignment.cpp
  src/obvision/registration/icp/assign/filter/DistanceFilter.cpp
  src/obvision/registration/icp/assign/filter/OcclusionFilter.cpp
  src/obvision/registration/icp/assign/filter/OutOfBoundsFilter2D.cpp
  src/obvision/registration/icp/assign/filter/ReciprocalFilter.cpp
  src/obvision/registration/icp/assign/filter/RobotFootprintFilter.cpp
  src/obvision/registration/icp/assign/filter/TrimmedFilter.cpp
  src/obvision/registration/ransacMatching/PDFMatching.cpp
  src/obvision/registration/ransacMatching/RandomMatching.cpp
  src/obvision/registration/ransacMatching/RandomNormalMatching.cpp
  src/obvision/registration/ransacMatching/TSD_PDFMatching.cpp
  src/obvision/registration/ransacMatching/TwinPointMatching.cpp
)
target_include_directories(slam_node
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>
  PRIVATE $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)
target_compile_features(slam_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_directories(slam_node
  PRIVATE /usr/local/lib
)
ament_target_dependencies(slam_node
  rclcpp
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(slam_node
  gsl
  gslcblas
  ann
  flann
  Boost::thread
  OpenMP::OpenMP_CXX
  lz4
)
rosidl_target_interfaces(slam_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install node(s)
install(TARGETS slam_node
  DESTINATION lib/${PROJECT_NAME})

# Install all launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install all config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

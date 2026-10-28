cmake_minimum_required(VERSION 3.16)
project(rgbd_slam)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 외부 인터페이스용 커스텀 메시지. 위치 m, 자세각 deg, 차체 FRD, ZYX 순.
# nav_msgs/Odometry 는 쿼터니언 필드뿐이라 오일러각을 담을 자리가 없다.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CurrentPose.msg"
  DEPENDENCIES std_msgs
)

# ros-jazzy-gtsam 4.2.0 을 쓴다. 자체 빌드 GTSAM 은 rtabmap 과 ABI 가 충돌하므로 금지.
find_package(GTSAM REQUIRED)

set(RGBD_SLAM_DEPS
  rclcpp nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs rtabmap_msgs)

add_library(rgbd_slam_core SHARED
  src/dead_reckoning.cpp     # 휠 속도 + 자이로 적분 (DR)
  src/loop_source.cpp        # rtabmap /mapGraph 링크 + /info 시각 → DR 노드 매핑
  src/graph_optimizer.cpp    # GTSAM 팩터그래프 (iSAM2 + 루프 시 배치 LM)
  src/premeasure.cpp         # 시작 직후 정지 구간 사전 측정 (자이로 바이어스, 카메라 roll·pitch·높이)
)
target_include_directories(rgbd_slam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(rgbd_slam_core gtsam Eigen3::Eigen pthread)
ament_target_dependencies(rgbd_slam_core ${RGBD_SLAM_DEPS})

add_executable(slam_node src/slam_node.cpp)
target_link_libraries(slam_node rgbd_slam_core gtsam)
ament_target_dependencies(slam_node ${RGBD_SLAM_DEPS})

# 같은 패키지에서 만든 메시지를 쓰려면 타입서포트 타깃을 직접 링크해야 한다.
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(slam_node "${cpp_typesupport_target}")

install(TARGETS rgbd_slam_core slam_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)

ament_package()

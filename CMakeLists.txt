cmake_minimum_required(VERSION 2.8.3)
project(rectangular_table_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS pcl pcl_ros roscpp sensor_msgs geometry_msgs message_generation)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
	FILES
	Plane.msg
	PlaneStamped.msg
	PlaneArray.msg
)


## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
	 LIBRARIES rectangular_table_detection
	 CATKIN_DEPENDS pcl pcl_ros roscpp sensor_msgs geometry_msgs message_runtime
	 DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_filtering src/simple_filtering.cpp)

add_executable(ransac_plane_detection src/ransac_plane_detection.cpp src/concave_hull.cpp src/ransac_plane_compute.cpp src/rviz_publish.cpp src/group_matrix.cpp src/tableDetectionGeometricModel.cpp)

add_executable(detect_normal_planes src/detect_normal_planes.cpp src/ransac_plane_compute.cpp)
add_executable(rviz_plane_def_pub src/rviz_plane_def_pub.cpp src/rviz_publish.cpp)
#add_executable(test_union_find src/group_matrix.cpp)

target_link_libraries(simple_filtering
  ${catkin_LIBRARIES}
)
target_link_libraries(ransac_plane_detection
  ${catkin_LIBRARIES}
)
target_link_libraries(detect_normal_planes
  ${catkin_LIBRARIES}
)

target_link_libraries(rviz_plane_def_pub
  ${catkin_LIBRARIES}
)
#target_link_libraries(test_union_find
#  ${catkin_LIBRARIES}
#)

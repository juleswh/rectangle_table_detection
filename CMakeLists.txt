cmake_minimum_required(VERSION 2.8.3)
project(rectangular_table_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS pcl pcl_ros roscpp sensor_msgs)

catkin_package(
   INCLUDE_DIRS include
	 LIBRARIES rectangular_table_detection
	 CATKIN_DEPENDS pcl pcl_ros roscpp sensor_msgs
	 DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_filtering src/simple_filtering.cpp)
add_executable(ransac_plane_detection src/ransac_plane_detection.cpp src/concave_hull.cpp src/ransac_plane_compute.cpp src/rviz_publish.cpp src/group_matrix.cpp src/geometry_utilities.cpp)
#add_executable(test_union_find src/group_matrix.cpp)

target_link_libraries(simple_filtering
  ${catkin_LIBRARIES}
)
target_link_libraries(ransac_plane_detection
  ${catkin_LIBRARIES}
)
#target_link_libraries(test_union_find
#  ${catkin_LIBRARIES}
#)

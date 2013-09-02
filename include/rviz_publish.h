#ifndef _RVIZ_PUBLISH_H_
#define _RVIZ_PUBLISH_H_
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

/**
 * publish an rviz shape for displaying the detected table on rviz
 **/
void publish_rviz_shape(ros::Publisher* marker_pub, float dim_x, float dim_y, float dim_z=0.1, bool remove_shape = false);

#endif // _RVIZ_PUBLISH_H_

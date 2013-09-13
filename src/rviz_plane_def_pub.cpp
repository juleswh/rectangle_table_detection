#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "ransac_plane_compute.h"
#include <rectangular_table_detection/PlaneArray.h>
#include "rviz_publish.h"

#define DEFAULT_NODE_NAME "rviz_plane_def_pub"

double normal_angle_deviation;
double model_distance_max;
int plane_min_inliers;
int planes_to_detect;
std::string reference_tf_name;
int param_interval;

bool param_publish_plane_pcl;
bool param_publish_outliers_pcl;


double min_height_plane;
double max_height_plane;

ros::Publisher marker_array_pub;
ros::NodeHandle* nh;


void planes_cb (const rectangular_table_detection::PlaneArray& plane_array)
{
	publish_rviz_plane_norms(marker_array_pub, plane_array, plane_array.header.frame_id);
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, DEFAULT_NODE_NAME);
	nh = new ros::NodeHandle("~");

	marker_array_pub = nh->advertise<visualization_msgs::MarkerArray> ("visualization_msgs",1);

	ros::Subscriber sub = nh->subscribe ("input", 1, planes_cb);

	

	// Spin
	ros::spin ();
#ifdef DO_TIME_SPEC
	time_output.close();
#endif
}


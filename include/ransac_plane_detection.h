/**
 * \file ransac_plane_detection.h
 * The main file header for the ransac_plane_detection node
 */


#ifndef _RANSAC_PLANE_DETECTION_H
#define _RANSAC_PLANE_DETECTION_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/common/centroid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/extract_indices.h>
#define _USE_MATH_DEFINES
#include <math.h>

#include "concave_hull.h"
#include "ransac_plane_compute.h"
#include "rviz_publish.h"
#include "group_matrix.h"
#include "geometry_utilities.h"

//a debug utility
#define _PRINT_HERE_(s) ROS_DEBUG_STREAM("in " << __FILE__ << " line " << __LINE__ << "\t-\t" << s)

/** \name ROS-specific objects
 **/
///@{
ros::NodeHandle* nh;
/** publisher for the output point cloud representing the detected table
 **/
ros::Publisher output_pcl_pub;
/** publisher for the output point cloud representing the hull of the detected table.
 **/
ros::Publisher hull_pcl_pub;
/** publisher for the dimensions of the detected table.
 * Publishes a geometry_msg::Point message with x,y representing the size of the table
 **/
ros::Publisher dimensions_publisher;
/** Publisher for the rviz marker representing the table
 **/
ros::Publisher marker_pub;
///@}

/**
 * \name User-set Parameters
 * These variables can be set by user to control behavior of the program
 * \see \link sec_detection_param ROS node description \endlink
 **/
///@{
std::string reference_tf_name="/world";
double ransac_model_epsilon=0.05;
double ransac_dist_threshold=0.01;

double max_height_plane=0.9;
double min_height_plane=0.65;

bool param_publish_rviz_marker=false;
bool param_publish_plane_pcl=false;
bool param_publish_hull_pcl=false;

/** threshold to consideer cos ~= 0.
 * an angle with a cosinus <= this value will be considerated to represent an orthogonal angle (i.e. approx pi/2)
 **/
double param_cos_ortho_tolerance=0.08;
int param_interval=1;
///@}

void get_parameters();

/** The callback where we call for all other functions and do all the computation.
 */
void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input);

/** Main function.
 * Initialization of ROS node, topics, parameters, etc...
 */
int main (int argc, char** argv);


#endif // _RANSAC_PLANE_DETECTION_H

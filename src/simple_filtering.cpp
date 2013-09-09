/**
 * Source of the node for downsampling a point cloud.
 * \file simple_filtering.cpp
 * \author Jules Waldhart
 *
 * From PCL tutorials.
 */
#include <ros/ros.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

///default value for the leaf size parameter
#define DEFAULT_LEAF_SIZE 0.01

ros::Publisher pub;
///the side length of a leaf for the downsampling method
double param_leaf_size;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	ROS_INFO("cloud_cb");
	ROS_INFO_STREAM("filt input height" << cloud->height);
  sensor_msgs::PointCloud2 cloud_filtered;

  // Perform the actual filtering
  pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (param_leaf_size, param_leaf_size, param_leaf_size);
  sor.filter (cloud_filtered);

  // Publish the data
  pub.publish (cloud_filtered);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "simple_filtering");
  ros::NodeHandle nh("~");

	nh.param("leaf_size",param_leaf_size,DEFAULT_LEAF_SIZE);

	ROS_INFO("starting");
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}

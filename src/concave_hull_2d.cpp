#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

#include "concave_hull.h"

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr seg_concave_hull(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  cloud);

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input){
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>), 
		cloud_hull;
	pcl::fromROSMsg(*input, *cloud);
	sensor_msgs::PointCloud2Ptr output_msg (new sensor_msgs::PointCloud2);

	cloud_hull = seg_concave_hull(cloud);

	pcl::toROSMsg(*cloud_hull, *output_msg);
	pub.publish(*output_msg);
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr seg_concave_hull(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  cloud)
{
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>), 
		cloud_projected (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PCDReader reader;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGBA>);


/*	// Build a filter to remove spurious NaNs
	pcl::PassThrough<pcl::PointXYZRGBA> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0, 1.1);
	pass.filter (*cloud_filtered);
	ROS_DEBUG_STREAM("PointCloud after filtering has: "
		<< cloud_filtered->points.size () << " data points." << std::endl);
*/
	cloud_filtered = cloud;
	ROS_DEBUG_STREAM("PointCloud after filtering has: "
		<< cloud_filtered->points.size () << " data points." << std::endl);


	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (cloud_filtered);
	seg.segment (*inliers, *coefficients);
	std::cerr << "PointCloud after segmentation has: "
		<< inliers->indices.size () << " inliers." << std::endl 
		<< "coefficients : " << *coefficients;
	if (inliers->indices.size () == 0 ){
		ROS_WARN("no plane found");
		return cloud_hull;
	}

	return calc_concave_hull(cloud_filtered, inliers, coefficients);

}


int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "concave_hull_2d");
	ros::NodeHandle nh;

	ROS_INFO("starting cocave hull 2d");
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

	// Spin
	ros::spin ();
	return 0;
}

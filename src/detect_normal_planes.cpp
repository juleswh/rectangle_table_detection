/**\file detect_normal_planes.cpp
 * 
 * Main file of the node detect_normal_planes that detect (all) planes that are normal to a given vector, in a given height range.
 *
 * Actually, the user can define some limits for the plane detection:
 * -a minimal number of inliers of the plane, so too small planes are not detected.
 * -a number of planes to detect
 * When the first of the above limition is met, the program stops and waits for an other input point cloud.
 *
 * The method used is pcl ransac method, and it detects geometrical planes; points can be not grouped,
 * or even not define a physical plane.
 */

/**
 * \mainpage Node detect_normal_planes
 * This documentation presents the node detect_normal_planes. In this page you'll find it's usage, and on other
 * pages the documentation of the code.
 *
 * \section Overview
 * This node computes it's incoming point cloud data with a ransac method to find planes in the data.
 * The planes it searchs for have to be normal to the 'z' axis of the given reference frame (from tf).
 * The user can set a few parameters so the program can adapt to different cases. These parameters are
 * described in the next section.
 * 
 * \section Parameters
 *
 * \par \p plane_min_inliers (int, default: 100)
 * Indicates the minimum number of points that a plane must contain to be considered as valid.
 *
 * \par \p planes_to_detect (int, default: 0)
 * Indicates the max number of planes to seek. 0 means no limit.
 *
 * \par \p reference_tf (string, default: "/map")
 * The name of the tf frame to use as reference. This frame indicates both the vertical direction,
 * which is it's 'z' axis, and the height reference : it's origin is at an height of 0.
 * The node seek planes normal to this vertical reference.
 *
 * \par \p model_distance_max (float, default: 0.05)
 * The max distance of a point to the ransac model.
 *
 * \par \p normal_angle_deviation (float, default: 0.05)
 * The max tolerance over the angle between the plane and the given reference. (in radians)
 *
 * \par \p max_height_plane (float, default: 0)
 * \par \p min_height_plane (float, default: 0)
 * This two parameters indicate in which height range we should search for planes.
 * All points that are not in this range are removed before the search, so this can 
 * speed the process if you have lots of points and you are only seeking planes
 * in a given range.
 *
 * \par \p publish_plane_pcl (boolean, default: false)
 * Choose to publish a point cloud that contains all the planes.
 *
 * \par \p publish_outliers_pcl (boolean, default: false)
 * Choose to publish a point cloud of the outliers (point that do not belong to any detected plane)
 *
 * \par \p incoming_pcl_sampling_period (int, default: 1)
 * The node can compute only some point clouds, usefull if your table is not moving too fast and
 * your CPU time is precious. So this is the sampling period.
 * Warning! 0 will cause you some issues...
 *
 * \section Subscribed topics
 * \par \p ~input (sensor_msgs/PointCloud2)
 * The point cloud to process.
 * 
 * \section Published topics
 * \par \p ~planes_detected (rectangular_table_detection/PlaneArray)
 * The points and normals of the detected points.
 *
 * \par \p ~output (sensor_msgs/PointCloud2)
 * (optional) a point cloud that contains all the planes.
 *
 * \par \p ~outliers
 * (optional) a point cloud of the outliers (points that do not belong to any detected plane)
 *
 */
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl/common/centroid.h>

#include "ransac_plane_compute.h"
#include <rectangular_table_detection/PlaneArray.h>

#define DEFAULT_NODE_NAME "detect_normal_planes"
#define DEFAULT_PLANE_MIN_INLIERS 100
#define DEFAULT_PLANES_TO_DETECT 0
#define DEFAULT_REFERENCE_TF "/map"
#define DEFAULT_MODEL_DISTANCE_MAX 0.05
#define DEFAULT_NORMAL_ANGLE_DEVIATION 0.05

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

ros::Publisher output_pcl_pub;
ros::Publisher output_pcl_outliers_pub;
ros::Publisher planes_publisher;
ros::NodeHandle* nh;


void get_vertical_referecence(tf::Vector3& vertical,tf::Vector3& origin, const std::string& reference_tf_name, const std::string& camera_tf_name){
	static tf::TransformListener tf_listener;
	tf::StampedTransform cam_world_tf;
	tf::Vector3 z(0,0,1);

	//get the transform from the camera to the world to get the vertical axis
	try{
		tf_listener.lookupTransform(camera_tf_name, reference_tf_name,  
				ros::Time(0), cam_world_tf);
	}
	catch (tf::TransformException ex){
		ROS_WARN("%s",ex.what());
		return;
	}
	//compute the world z axis
	//the matrix representing the rotation of the transform
	tf::Matrix3x3 rot_matrix(cam_world_tf.getRotation());

	tf::Vector3 vert_camera(rot_matrix[0].x()*z.x() + rot_matrix[0].y()*z.y() + rot_matrix[0].z()*z.z(),
			rot_matrix[1].x()*z.x() + rot_matrix[1].y()*z.y() + rot_matrix[1].z()*z.z(),
			rot_matrix[2].x()*z.x() + rot_matrix[2].y()*z.y() + rot_matrix[2].z()*z.z());

	vertical = vert_camera;
	origin = cam_world_tf.getOrigin();
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//ignore some point clouds, according to parameter sample period
	static size_t compteur = 0;
	if( (compteur++ % param_interval) != 0){
		ROS_DEBUG("skipping this point cloud");
		return;
	}
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());

	tf::StampedTransform cam_world_tf;
	tf::Vector3 z(0,0,1); //z axis

	pcl::IndicesPtr points_to_consider;

	//output values
	Eigen::VectorXf coefficients;
	pcl::IndicesPtr plane_inliers(new std::vector<int>);
	pcl::IndicesPtr all_planes_inliers(new std::vector<int>);

	rectangular_table_detection::PlaneArray plane_array;

	bool continue_searching=true;//for the loop condition

	std::vector<bool> is_point_to_compute;

	//get the vertical reference of the world in the camera coordinates
	tf::Vector3 vert_camera,origin_world_camera;
	get_vertical_referecence(vert_camera,origin_world_camera, reference_tf_name,input->header.frame_id);
	if(vert_camera.length()<=0.01){
		ROS_WARN("no vertical reference");
		return;
	}
	//from these, create a line that will be used to get the height of the plane we find
	Eigen::ParametrizedLine<float,3> vertical_line(Eigen::Vector3f(origin_world_camera[0],origin_world_camera[1],origin_world_camera[2]),Eigen::Vector3f(vert_camera[0],vert_camera[1],vert_camera[2]));

	// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl::fromROSMsg (*input, *cloud);
	if(cloud->size()<=1){
		ROS_WARN("no points in input cloud");
		return;
	}
	ROS_DEBUG("cloud size %d",cloud->size());

	//selects points in the cloud that will be computed
	if(max_height_plane - min_height_plane == 0.){
		//select all
		is_point_to_compute.resize(cloud->size(),true);
	}else{
		ROS_INFO("remove out of range");
		is_point_to_compute.resize(cloud->size(),false);

		//select only points that are in the height range
		filter_out_of_range_points(cloud,points_to_consider,vertical_line,min_height_plane,max_height_plane);
		
		//set to true the selected points
		for(std::vector<int>::iterator it=points_to_consider->begin(); it!=points_to_consider->end() ; ++it){
			is_point_to_compute[*it]=true;
		}
	}

	//search
	while(continue_searching){
		ROS_INFO_STREAM("iteration "<<plane_array.planes.size());
		rectangular_table_detection::Plane plane_msg;
		//search for the plane in the selected range
		ransac_plane_compute(cloud,vertical_line,normal_angle_deviation,model_distance_max,points_to_consider, plane_inliers, coefficients, min_height_plane,max_height_plane);

		if (plane_inliers->size() < plane_min_inliers){
			ROS_DEBUG("too small plane : %d", plane_inliers->size());
			continue_searching=false;
		}else{
			ROS_DEBUG("plane found");

			all_planes_inliers->insert(all_planes_inliers->end(),plane_inliers->begin(),plane_inliers->end());

			Eigen::Vector3f normal(coefficients[0],coefficients[1],coefficients[2]);
			Eigen::Vector4f centroid;
			pcl::compute3DCentroid(*cloud, *plane_inliers,centroid);

			//flip the normal so it's in the same direction than vertical line direction (if necessary)
			if(normal.dot(vertical_line.direction()) < 0){
				normal[0] = -normal[0];
				normal[1] = -normal[1];
				normal[2] = -normal[2];
			}

			//add the plane to the msg array
			plane_msg.point.x=centroid[0];
			plane_msg.point.y=centroid[1];
			plane_msg.point.z=centroid[2];

			plane_msg.normal.x=normal[0];
			plane_msg.normal.y=normal[1];
			plane_msg.normal.z=normal[2];

			plane_array.planes.push_back(plane_msg);

			//test if we should continue
			//stop if there is a limit of planes to detect and this limit is reached
			if((planes_to_detect!=0) && (plane_array.planes.size() >= planes_to_detect)){
				continue_searching=false;
			}else{
				ROS_INFO("remove inliers");
				//remove the plane inliers from the set of points to compute
				for(std::vector<int>::iterator it=plane_inliers->begin(); it!=plane_inliers->end() ; ++it){
					is_point_to_compute[*it]=false;
				}
				points_to_consider = pcl::IndicesPtr(new std::vector<int>);
				for(std::vector<bool>::iterator it=is_point_to_compute.begin(); it!=is_point_to_compute.end() ; ++it){
					if(*it){
						points_to_consider->push_back(it - is_point_to_compute.begin());
					}
				}
			}
		}
	}

	ROS_INFO("found %d planes",plane_array.planes.size());

	//publish the results
	plane_array.header.frame_id = input->header.frame_id;
	planes_publisher.publish(plane_array);

}


int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, DEFAULT_NODE_NAME);
	nh = new ros::NodeHandle("~");

	nh->param("plane_min_inliers", plane_min_inliers, DEFAULT_PLANE_MIN_INLIERS);
	nh->param("planes_to_detect", planes_to_detect, DEFAULT_PLANES_TO_DETECT);

	//get or set defaults values for parameters
	nh->param<std::string>("reference_tf", reference_tf_name, DEFAULT_REFERENCE_TF);
	nh->param("model_distance_max", model_distance_max, DEFAULT_MODEL_DISTANCE_MAX);
	nh->param("normal_angle_deviation", normal_angle_deviation, DEFAULT_NORMAL_ANGLE_DEVIATION);

	nh->param("max_height_plane", max_height_plane, 0.);
	nh->param("min_height_plane", min_height_plane, 0.);

	//all publish options are set to false by default
	nh->param("publish_plane_pcl", param_publish_plane_pcl, false);
	nh->param("publish_outliers_pcl", param_publish_outliers_pcl, false);

	nh->param("incoming_pcl_sampling_period", param_interval, 1);

	ROS_INFO("starting ransac orthogonal planes detection");
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh->subscribe ("input", 1, cloud_cb);

	// Create a ROS publisher for the output point cloud
	if(param_publish_plane_pcl)
		output_pcl_pub = nh->advertise<sensor_msgs::PointCloud2> ("output", 1);
	if(param_publish_outliers_pcl)
		output_pcl_outliers_pub = nh->advertise<sensor_msgs::PointCloud2> ("outliers",1);

	planes_publisher = nh->advertise<rectangular_table_detection::PlaneArray> ("planes_detected",1);
	

	// Spin
	ros::spin ();
#ifdef DO_TIME_SPEC
	time_output.close();
#endif
}

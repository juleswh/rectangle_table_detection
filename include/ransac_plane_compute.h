/**
 * File containing the function to find a normal plane with a ransac method, within a defined height interval.
 * \file ransac_plane_compute.h
 **/
#ifndef _RANSAC_PLANE_COMPUTE_H_
#define _RANSAC_PLANE_COMPUTE_H_

#include <ros/ros.h>
#include <tf/tf.h>

// PCL specific includes
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


//a debug utility
#define _PRINT_HERE_(s) ROS_DEBUG_STREAM("in " << __FILE__ << " line " << __LINE__ << "\t-\t" << s)


/**
 * compute ransac over the point cloud to find a plan normal to the vertical and in the height interval.
 * \param[in] cloud the cloud
 * \param[in] vert_camera a parametrized line representing the axis to use as the normal of the plan and the point at a 0 height
 * \param[in] ransac_model_epsilon the epsilon value for the ransac plane model
 * \param[in] ransac_dist_threshold the maximum distance to the ransac plane model
 * \param[out] inliers the inliers indices of the plane found
 * \param[out] coefficients the coefficients of the plane found
 * \param[in] min_height_plane the minimum height of the plane to search for
 * \param[in] max_height_plane the maximum height of the plane to search for
 *
 * this function firstly selects all the points that are in the given range, then search for a plane definition with RANSAC method
 * within this points selection.
 **/

void ransac_plane_compute(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, Eigen::ParametrizedLine<float,3> vert_camera, double ransac_model_epsilon, double ransac_dist_threshold, pcl::IndicesPtr inliers, Eigen::VectorXf& coefficients, double min_height_plane, double max_height_plane);

void filter_out_of_range_points(
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, 
		std::vector<int>& inliers,
		Eigen::ParametrizedLine<float,3> vertical_param_line_camera, 
		double min_height_plane, double max_height_plane);

bool is_point_in_user_range(Eigen::ParametrizedLine<float, 3> line_vertical, pcl::PointXYZRGBA point_of_plane, double min_height_plane, double max_height_plane);

#endif // _RANSAC_PLANE_COMPUTE_H_

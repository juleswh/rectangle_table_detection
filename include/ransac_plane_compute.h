/**
 * File containing the function to find a normal plane with a ransac method, within a defined height interval.
 * \file ransac_plane_compute.h
 * \author Jules Waldhart
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
 * this function search for a plane definition with RANSAC method
 **/

void ransac_plane_compute(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, Eigen::ParametrizedLine<float,3> vert_camera, double ransac_model_epsilon, double ransac_dist_threshold, pcl::IndicesPtr inliers, Eigen::VectorXf& coefficients, double min_height_plane, double max_height_plane);

/**
 * compute ransac over the points of the point cloud indicated by indices_to_use to find a plane
 * normal to the vertical and in the given height range
 * \param[in] cloud the cloud
 * \param[in] vert_camera a parametrized line representing the axis to use as the normal of the plan and the point at a 0 height
 * \param[in] ransac_model_epsilon the epsilon value for the ransac plane model
 * \param[in] ransac_dist_threshold the maximum distance to the ransac plane model
 * \param[out] inliers the inliers indices of the plane found
 * \param[out] coefficients the coefficients of the plane found
 * \param[in] min_height_plane the minimum height of the plane to search for
 * \param[in] max_height_plane the maximum height of the plane to search for
 * \param[in] indices_to_use the point indices to use in the cloud (if pointer is empty, takes all the point cloud)
 *
 * this function search for a plane definition with RANSAC method
 * within this points selection.
 * \warning the parameter indices_to_use is ignored if the pointer is empty, not if the vector is empty.
 * \code{.cpp}
 * if(indices_to_use)
 * 	//use only points indicated by *indices_to_use
 * }else{
 * 	//use all the point cloud
 * }
 * \endcode
 * 
 */
void ransac_plane_compute(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, Eigen::ParametrizedLine<float,3> vert_camera, double ransac_model_epsilon, double ransac_dist_threshold, pcl::IndicesConstPtr indices_to_use, pcl::IndicesPtr inliers, Eigen::VectorXf& coefficients, double min_height_plane, double max_height_plane);

/**
 * select points that are in the heigh range.
 * \param[in] cloud
 * \param[out] indices
 * \param[in] vertical_param_line_camera
 * \param[in] min_height_plane
 * \param[in] max_height_plane
 *
 *
 * Projects each point over the line \c vertical_param_line_camera and if the height
 * (distance between origin of the line and the projected point) is in the range, 
 * we add it to the indices array.
 *
 * \note \c indices can be empty, we create a new array.
 */
void filter_out_of_range_points(
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, 
		pcl::IndicesPtr& indices,
		Eigen::ParametrizedLine<float,3> vertical_param_line_camera, 
		double min_height_plane, double max_height_plane);

bool is_point_in_user_range(Eigen::ParametrizedLine<float, 3> line_vertical, pcl::PointXYZRGBA point_of_plane, double min_height_plane, double max_height_plane);

#endif // _RANSAC_PLANE_COMPUTE_H_

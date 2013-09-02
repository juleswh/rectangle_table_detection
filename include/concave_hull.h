/**
 * 2D hull computation utilities for point clouds.
 * \file concave_hull.h
 * \author Jules Waldhart
 *
 * Utilities using pcl::ConcaveHull, pcl::ConvexHull to computer 2D hulls of 2D point clouds.
 * Works on XYZRGBA (kinect-like, RGB + Depth) point clouds
 *
 */

#ifndef _CONCAVE_HULL_H_
#define _CONCAVE_HULL_H_
#include <ros/ros.h>

// PCL specific includes
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/sample_consensus/ransac.h>

#define _PRINT_HERE_(s) ROS_DEBUG_STREAM("in " << __FILE__ << " line " << __LINE__ << "\t-\t" << s)

/**
 * Compute concave hull of a plan.
 * \param[in] cloud the complete cloud
 * \param[in] inliers indices of points to compute; indicates plan inliers
 * \param[in] coefficients the coefficients of the plan
 * \return the hull as a PointCloud of points representing the plan's hull
 *
 * This function firstly project inliers over the plan indicated by it's coefficients,
 * and then compute the hull of these projected points
 *
 * \todo TODO add parameter for alpha
 *
 **/
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr calc_concave_hull(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients);

/**
 * As calc_concave_hull(), but for convex hull
 * \see calc_concave_hull()
 **/
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr calc_convex_hull(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients);


/**
 * Search a line in a point cloud with RANSAC method.
 * \param[in] plane_hull the point cloud to use
 * \param[in] dist_thresh a parameter for the ransac method
 * \param[out] inliers the indices of the points belonging to the line
 * \param[out] coefficients the coefficients of the line
 * \see http://docs.pointclouds.org/trunk/classpcl_1_1_random_sample_consensus.html
 **/
void find_line_ransac(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr plane_hull, const double dist_thresh, pcl::IndicesPtr inliers, Eigen::VectorXf& coefficients);

#endif //_CONCAVE_HULL_H_

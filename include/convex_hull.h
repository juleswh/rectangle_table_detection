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
#include <pcl/surface/convex_hull.h>
#include <pcl/sample_consensus/ransac.h>

#define _PRINT_HERE_(s) ROS_DEBUG_STREAM("in " << __FILE__ << " line " << __LINE__ << "\t-\t" << s)

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr calc_convex_hull(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients);


void find_line_ransac(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr plane_hull, const double dist_thresh, pcl::IndicesPtr inliers, Eigen::VectorXf& coefficients);

#endif //_CONCAVE_HULL_H_

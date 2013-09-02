#include "convex_hull.h"

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr calc_convex_hull(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients){
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGBA>),
		cloud_projected (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::ConvexHull<pcl::PointXYZRGBA> chull;
	// Project the model inliers
	pcl::ProjectInliers<pcl::PointXYZRGBA> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setIndices (inliers);
	proj.setInputCloud (cloud);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cloud_projected);

	// Create a Convex Hull representation of the projected inliers
	chull.setInputCloud (cloud_projected);
	//chull.setAlpha (0.1);
	chull.reconstruct (*cloud_hull);

	return cloud_hull;
}

void find_line_ransac(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr plane_hull, const double dist_thresh, pcl::IndicesPtr inliers, Eigen::VectorXf& coefficients){
	pcl::SampleConsensusModelLine<pcl::PointXYZRGBA>::Ptr line_model ( new pcl::SampleConsensusModelLine<pcl::PointXYZRGBA> (plane_hull));
	pcl::RandomSampleConsensus<pcl::PointXYZRGBA> line_ransac (line_model);
	line_ransac.setDistanceThreshold(dist_thresh);
	//compute
	line_ransac.computeModel();
	//get results
	line_ransac.getInliers(*inliers);
	line_ransac.getModelCoefficients(coefficients);
}

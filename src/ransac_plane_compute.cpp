#include "ransac_plane_compute.h"

bool is_point_in_user_range(Eigen::ParametrizedLine<float, 3> line_vertical, pcl::PointXYZRGBA point_of_plane, double min_height_plane, double max_height_plane){
	bool is_ok = false;
	Eigen::Vector3f projection_on_line = line_vertical.projection(Eigen::Vector3f(point_of_plane.x,point_of_plane.y,point_of_plane.z));
	Eigen::Vector3f vert_to_point = - line_vertical.origin() + projection_on_line;
	if ( (vert_to_point.norm() < max_height_plane) && (vert_to_point.norm() > min_height_plane)){
		is_ok=true;
	}
	return is_ok;

}


void filter_out_of_range_points(
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, 
		std::vector<int>& inliers,
		Eigen::ParametrizedLine<float,3> vertical_param_line_camera, 
		double min_height_plane, double max_height_plane)
{
	for (int i = 0; i<cloud->size() ; i++){
		if(is_point_in_user_range(vertical_param_line_camera,cloud->at(i),min_height_plane,max_height_plane)){
			inliers.push_back(i);
		}
	}
}
			

void ransac_plane_compute(
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, 
		Eigen::ParametrizedLine<float,3> vertical_param_line_camera, 
		double ransac_model_epsilon,
	 	double ransac_dist_threshold,
	 	pcl::IndicesPtr inliers, 
		Eigen::VectorXf& coefficients, 
		double min_height_plane, double max_height_plane)
{

	bool ransac_plane_ok=false;
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

	//ROS_DEBUG_STREAM("before filter out of range " << cloud->size());
	pcl::IndicesPtr filter_inliers(new std::vector<int>);
	filter_out_of_range_points(cloud,*filter_inliers,vertical_param_line_camera,min_height_plane,max_height_plane);
	extract.setInputCloud (cloud);
	extract.setIndices (filter_inliers);
	extract.setNegative (false);
	extract.filter (*cloud);
	//ROS_DEBUG_STREAM("filter out of range " << cloud->size());

	while(!ransac_plane_ok){
		pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZRGBA>::Ptr model ( new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZRGBA> (cloud));
		model->setAxis(vertical_param_line_camera.direction());
		model->setEpsAngle(ransac_model_epsilon);
		pcl::RandomSampleConsensus<pcl::PointXYZRGBA> ransac (model);
		ransac.setDistanceThreshold(ransac_dist_threshold);
		//compute
		ransac.computeModel();
		//get results
		ransac.getInliers(*inliers);
		ransac.getModelCoefficients(coefficients);

		if (inliers->size () == 0)
		{
			ROS_WARN ("Could not estimate a planar model for the given dataset.");
			return;
		}

		//if the plane we found is not is the height range defined, we remove the points and start again
		if(!is_point_in_user_range(vertical_param_line_camera,(*cloud)[(*inliers)[0]],min_height_plane,max_height_plane)){
			ROS_DEBUG_STREAM("not the good plane");
			extract.setInputCloud (cloud);
			extract.setIndices (pcl::IndicesPtr(inliers));
			extract.setNegative (true);
			extract.filter (*cloud);
		}else{
			ransac_plane_ok=true;
		}
	}
}

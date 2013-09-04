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
		pcl::IndicesPtr indices,
		Eigen::ParametrizedLine<float,3> vertical_param_line_camera, 
		double min_height_plane, double max_height_plane)
{
	indices = pcl::IndicesPtr(new std::vector<int>);
	for (int i = 0; i<cloud->size() ; i++){
		if(is_point_in_user_range(vertical_param_line_camera,cloud->at(i),min_height_plane,max_height_plane)){
			indices->push_back(i);
		}
	}
}


void ransac_plane_compute(
		pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, 
		Eigen::ParametrizedLine<float,3> vertical_param_line_camera, 
		double ransac_model_epsilon,
	 	double ransac_dist_threshold,
	 	pcl::IndicesPtr inliers, 
		Eigen::VectorXf& coefficients, 
		double min_height_plane, double max_height_plane)
{
	ransac_plane_compute(cloud, vertical_param_line_camera, ransac_model_epsilon, ransac_dist_threshold,
			pcl::IndicesConstPtr(), inliers, coefficients, min_height_plane, max_height_plane);
}

void ransac_plane_compute(
		pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, 
		Eigen::ParametrizedLine<float,3> vertical_param_line_camera, 
		double ransac_model_epsilon,
	 	double ransac_dist_threshold,
		pcl::IndicesConstPtr indices_to_use,
	 	pcl::IndicesPtr inliers, 
		Eigen::VectorXf& coefficients, 
		double min_height_plane, double max_height_plane)
{

	bool ransac_plane_ok=false;
	pcl::IndicesPtr indices_to_compute;
	if (indices_to_use){
		indices_to_compute = pcl::IndicesPtr(new std::vector<int>(*indices_to_use));
	}

	while(!ransac_plane_ok){
		pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZRGBA>::Ptr model ( new pcl::SampleConsensusModelPerpendicularPlane<pcl::PointXYZRGBA> (cloud));
		if (indices_to_compute) {
			model->setIndices(*indices_to_compute);
		}

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
			break;
		}

		//if the plane we found is not is the height range defined, we remove the points and start again
		if(!is_point_in_user_range(vertical_param_line_camera,(*cloud)[(*inliers)[0]],min_height_plane,max_height_plane)){
			ROS_DEBUG_STREAM("not the good plane");
			if(!indices_to_compute){
				//i.e. we were working on the whole point cloud
				//we create a new IndicesPtr
				indices_to_compute = pcl::IndicesPtr(new std::vector<int>);
				//we put in it all indices, except those in inliers
				for (int i=0; i<cloud->size();i++){
					//if the index is not in the inliers, we add it
					if (std::find(inliers->begin(),inliers->end(),i) == inliers->end()){
						indices_to_compute->push_back(i);
					}
				}
			}else{
				//if there was already a selection of points
				//we remove inliers from this selection
				for (int i=0; i<inliers->size(); i++){
					indices_to_compute->erase(std::remove(indices_to_compute->begin(), indices_to_compute->end(), (*inliers)[i]),indices_to_compute->end());
				}
			}
		}else{
			ransac_plane_ok=true;
		}
	}
}

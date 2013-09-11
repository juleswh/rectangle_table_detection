#include "ransac_plane_detection.h"

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
#ifdef DO_TIME_SPEC
	ros::Time begin = ros::Time::now();
#endif

	tf::Transform plane_tf;
	tf::StampedTransform cam_world_tf;
	Eigen::Vector4f centroid;
	Eigen::Vector3f plane_center;
	bool plane_center_found = false;
	static tf::TransformBroadcaster tf_br;

	tf::Vector3 z(0,0,1); //z axis

	//output values
	Eigen::VectorXf coefficients;
	pcl::IndicesPtr plane_inliers(new std::vector<int>);

	ROS_DEBUG("#################");
	ROS_DEBUG("cloud_cb");
	ROS_DEBUG("#################");

#ifdef DO_TIME_SPEC
	//init
	time_output << ros::Time::now()-begin << ",";
#endif

	//get the vertical reference of the world in the camera coordinates
	tf::Vector3 vert_camera,origin_world_camera;
	get_vertical_referecence(vert_camera,origin_world_camera, reference_tf_name,input->header.frame_id);
	if(vert_camera.length()<=0.01){
		ROS_WARN("no vertical reference");
		return;
	}
	//from these, create a line that will be used to get the height of the plane we find
	geometry_model.setVerticalLine(Eigen::Vector3f(origin_world_camera[0],origin_world_camera[1],origin_world_camera[2]),Eigen::Vector3f(vert_camera[0],vert_camera[1],vert_camera[2]));

#ifdef DO_TIME_SPEC
	//get vertical
	time_output << ros::Time::now()-begin << ",";
#endif

	// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
	pcl::fromROSMsg (*input, *cloud);
	if(cloud->size()<=1){
		ROS_WARN("no points in input cloud");
		return;
	}

#ifdef DO_TIME_SPEC
	//convert msg
	time_output << ros::Time::now()-begin << ",";
#endif

	//select only points that are in the height range
	pcl::IndicesPtr in_range_filter_inliers;
	filter_out_of_range_points(cloud,in_range_filter_inliers,geometry_model.getVerticalLine(),min_height_plane,max_height_plane);

#ifdef DO_TIME_SPEC
	//select range
	time_output << ros::Time::now()-begin << ",";
#endif

	//search for the plane in the selected range
	ransac_plane_compute(cloud,geometry_model.getVerticalLine(),ransac_model_epsilon,ransac_dist_threshold,in_range_filter_inliers, plane_inliers, coefficients, min_height_plane,max_height_plane);

#ifdef DO_TIME_SPEC
	//ransac plane
	time_output << ros::Time::now()-begin << ",";
#endif

	/////////////////////////////////////////////////////////
	//compute CONVEX HULL of the plane
	pcl::ModelCoefficientsPtr m_coefficients(new pcl::ModelCoefficients());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr projection;

	//convert coefficients to other format
	for(size_t i=0 ; i<coefficients.size() ; i++ ){
		m_coefficients->values.push_back(coefficients[i]);
	}

	/////////////////////////////////////////////////////////
	//Extract the inliers to an other point cloud
	
	pcl::PointIndices::Ptr p_plane_inliers(new pcl::PointIndices);
	pcl::PointIndices::Ptr table_indices(new pcl::PointIndices);
	p_plane_inliers->indices = *plane_inliers;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	std::vector<int> group_inliers;
	//compute the point cloud representing the plan (extract inliers)
	pcl::ExtractIndices<pcl::PointXYZRGBA> filt_extract;
	filt_extract.setInputCloud (cloud);
	filt_extract.setIndices (p_plane_inliers);
	filt_extract.setNegative (false);
	filt_extract.filter (*plane_cloud);

	//////////////////////////////////////////////////////////
	//find the biggest group of points (table_indices)
	union_find(plane_cloud,plane_cloud->size()/200, group_inliers);
	table_indices->indices = group_inliers;

#ifdef DO_TIME_SPEC
	//union-find
	time_output << ros::Time::now()-begin << ",";
#endif

	/////////////////////////////////////////////////////////
	//compute the hull of this biggest group
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr hull = calc_concave_hull(plane_cloud, table_indices, m_coefficients,concave_hull_alpha);

#ifdef DO_TIME_SPEC
	//hull
	time_output << ros::Time::now()-begin << ",";
#endif

	/////////////////////////////////////////////////////////
	//	compute RANSAC lines of the hull ////////////////////

	//std::vector<Line_def*> borders_param_lines;
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	//std::vector<Vertex_def*> table_vertices;
	tableDetectionGeometricModel::Rectangle table_rectangle;
	//a copy of the hull Pt Cld that will be modifided
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr borders_hull_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>(*hull));

	//finds all the lines and vertices
	size_t i=0;
	//while we have points in the point cloud (more than a certain percentage of the initial hull)
	while((borders_hull_cloud->size()>hull->size()*0.1) && (i < 10)){
		pcl::IndicesPtr line_inliers(new std::vector<int>);
		Eigen::VectorXf line_coeffs;

		find_line_ransac(borders_hull_cloud, ransac_line_dist_threshold, line_inliers,line_coeffs);

		//check if we found a line or not, if not, exits loop
		if (line_inliers->size() == 0) {
			ROS_WARN("found only %d vertices with %d lines", geometry_model.vertices_.size(),i);
			break;
		}

		//creates a parametrized line representing the border
		geometry_model.addBorder(line_coeffs);

		//finds vertices, defined as the intersection of 2 orthogonals edges
		for (size_t j=0 ; j < i ; j++){
			//TODO : correct bug when less than 4 vertices are found
			//ROS_DEBUG_STREAM("edges angle " << std::abs(geometry_model.borders[j].direction().dot(geometry_model.borders[i].direction())));
			if ( geometry_model.areBordersOrthogonal(i,j) ){
				//if they are orthogonals
				//we get a vertex of the table
				geometry_model.addVertexFromEdges(i,j);
			}
		}

		//remove points of the line we just found to find another one in the next iteration
		extract.setInputCloud (borders_hull_cloud);
		extract.setIndices (line_inliers);
		extract.setNegative (true);
		extract.filter (*borders_hull_cloud);

		i++;
	}

#ifdef DO_TIME_SPEC
	//borders detect
	time_output << ros::Time::now()-begin << ",";
#endif

	ROS_DEBUG("borders used %d",geometry_model.borders_.size());
	ROS_DEBUG("vertices found %d",geometry_model.vertices_.size());

#ifdef _DEBUG_FUNCTIONS_
	pcl::PointIndices::Ptr not_matched_points(new pcl::PointIndices);
#endif
	//if we have more than 4 vertices, it's ok, otherwise, we notify the user
	if(geometry_model.verticesCount() >= 4 ){
		geometry_model.find_all_possible_rectangles();

		ROS_DEBUG("possible rectangles = %d",geometry_model.possibleRectanglesCount());

		//select the best rectangle
		//let's play a match
#ifdef _DEBUG_FUNCTIONS_ //{
		float best_score;
		int best_index;
		int best_rectangle_index=geometry_model.select_best_matching_rectangle(plane_cloud,table_indices,0.80,0.,plane_cloud->size()/10,best_score,best_index,not_matched_points);

		if (best_rectangle_index >=0){
			//ROS_DEBUG("max score = %f, not matched : %d",best_score, not_matched_points->indices.size());
			table_rectangle = *geometry_model.possible_rectangles_[best_rectangle_index];
			plane_center_found = true;
		}else{
			ROS_WARN("did not find a good definition of the table");
			plane_center_found = false;
			//print a log
			std::ofstream debug_log_file;
			debug_log_file.open("debug_graph_file.tex",ios::app | ios::out);
			debug_log_file << "\\begin{figure}\n\\center\n";
			for (int i=0;i<geometry_model.borders_.size();i++){
				debug_log_file <<"\\begin{tikzpicture}\n";
				if(geometry_model.print_graph(geometry_model.borders_[i],0,debug_log_file) !=0 ){
					debug_log_file <<"\\end{tikzpicture}\noooooo\\\\ \n";
				}else{
					debug_log_file <<"\\end{tikzpicture}\n";
				}
			}
			debug_log_file << "\\caption{possible rectangles = " << geometry_model.possible_rectangles_.size()
				<< "; edges = " << geometry_model.borders_.size() 
				<< "; vertices = " << geometry_model.vertices_.size() 
				<< "best score = " << best_score
				<<", best dimensions " << geometry_model.possible_rectangles_[best_index]->vect_x.norm() 
				<< " " << geometry_model.possible_rectangles_[best_index]->vect_y.norm() 
				<< ", not matched " << not_matched_points->indices.size() 
				<< "}\n";
			debug_log_file << "\\end{figure}\n";
			debug_log_file.close();
		}
//}
#else //{

		int best_rectangle_index=geometry_model.select_best_matching_rectangle(plane_cloud,table_indices,0.80,0.,plane_cloud->size()/10);
		if (best_rectangle_index >=0){
			table_rectangle = *geometry_model.possible_rectangles_[best_rectangle_index];
			plane_center_found = true;
		}else{
			ROS_WARN("did not find a good definition of the table");
			plane_center_found = false;
		}
//}
#endif

	}else{
		ROS_WARN_STREAM("table not defined, "<< geometry_model.vertices_.size() << "vertices found");
		plane_center_found = false;
	}
	/////////////////////////////////////////////////////////

#ifdef DO_TIME_SPEC
	//best rect
	time_output << ros::Time::now()-begin << ",";
#endif


	if (plane_center_found){
		ROS_DEBUG("table correctly defined");

		//compute transform to place the plane with tf
		plane_tf = geometry_model.computeTransform (table_rectangle.point, table_rectangle.vect_x,table_rectangle.vect_y);

		//publish the dimensions
		geometry_msgs::Point dimensions_msg;
		dimensions_msg.x = table_rectangle.vect_x.norm();
		dimensions_msg.y = table_rectangle.vect_y.norm();
		dimensions_msg.z=0;
		dimensions_publisher.publish(dimensions_msg);
		tf_br.sendTransform(tf::StampedTransform(plane_tf, ros::Time::now(), "camera_depth_optical_frame", "/plane_segmented"));
	}

#ifdef DO_TIME_SPEC
	//pub dim
	time_output << ros::Time::now()-begin << ",";
#endif


	//publish the marker
	if (param_publish_rviz_marker){
		if(plane_center_found){
			publish_rviz_shape(&marker_pub,table_rectangle.vect_x.norm(),table_rectangle.vect_y.norm());
		}else{
			//remove if no plane found
			publish_rviz_shape(&marker_pub,0,0,0,true);
		}
	}
	//publish PointCloud of the plane (selects inliers in the PointCloud)
	if (param_publish_plane_pcl) {
		sensor_msgs::PointCloud2 msg_cloud_inliers;
		pcl::PointCloud<pcl::PointXYZRGBA> cloud_inliers;
		//for (size_t i = 0; i < plane_inliers->size (); ++i)
		//	cloud_inliers.insert(cloud_inliers.end(),cloud->points[(*plane_inliers)[i]]);
		for (size_t i = 0; i < table_indices->indices.size (); ++i)
			cloud_inliers.insert(cloud_inliers.end(),plane_cloud->points[(table_indices->indices)[i]]);

		// convert and Publish the data
		pcl::toROSMsg(cloud_inliers,msg_cloud_inliers);
		//modify the frame id
		msg_cloud_inliers.header.frame_id=input->header.frame_id;
		output_pcl_pub.publish (msg_cloud_inliers);
	}
#ifdef _DEBUG_FUNCTIONS_
	//publish PointCloud of the plane (selects inliers in the PointCloud)
	if (param_publish_plane_pcl) {
		sensor_msgs::PointCloud2 msg_cloud_inliers;
		pcl::PointCloud<pcl::PointXYZRGBA> cloud_inliers;
		//for (size_t i = 0; i < plane_inliers->size (); ++i)
		//	cloud_inliers.insert(cloud_inliers.end(),cloud->points[(*plane_inliers)[i]]);
		//for (size_t i = 0; i < table_indices->indices.size (); ++i)
		for (size_t i = 0; i < not_matched_points->indices.size (); ++i)
			//cloud_inliers.insert(cloud_inliers.end(),plane_cloud->points[(table_indices->indices)[i]]);
			cloud_inliers.insert(cloud_inliers.end(),plane_cloud->points[(not_matched_points->indices)[i]]);

		// convert and Publish the data
		pcl::toROSMsg(cloud_inliers,msg_cloud_inliers);
		//pcl::toROSMsg(*line1_cloud,msg_cloud_inliers);
		//modify the frame id
		msg_cloud_inliers.header.frame_id=input->header.frame_id;
		output_pcl_pub.publish (msg_cloud_inliers);
	}
#endif

	//publish the outliers point cloud
	if(param_publish_outliers_pcl){
		sensor_msgs::PointCloud2 msg_outliers;
		pcl::PointCloud<pcl::PointXYZRGBA> cloud_outliers;
		filt_extract.setInputCloud (cloud);
		filt_extract.setIndices (p_plane_inliers);
		filt_extract.setNegative (true);
		filt_extract.filter (cloud_outliers);
		pcl::toROSMsg(cloud_outliers, msg_outliers);
		msg_outliers.header.frame_id=input->header.frame_id;
		output_pcl_outliers_pub.publish(msg_outliers);
	}

	if(param_publish_hull_pcl){
		//publish the hull
		sensor_msgs::PointCloud2 msg_hull;
		pcl::toROSMsg(*hull,msg_hull);
		msg_hull.header.frame_id=input->header.frame_id;
		hull_pcl_pub.publish (msg_hull);
	}

	geometry_model.clear();

#ifdef DO_TIME_SPEC
	//other pub + end
	time_output << ros::Time::now()-begin << "\n";
	time_output.flush();
#endif
	
	return;
}

int main (int argc, char** argv)
{
	double ortho_rad_thresh;
	// Initialize ROS
	ros::init (argc, argv, "ransac_plane_detection");
	nh = new ros::NodeHandle("~");

	//get or set defaults values for parameters
	nh->param<std::string>("reference_tf", reference_tf_name, DEFAULT_REFERENCE_TF);
	nh->param("ransac_dist_threshold", ransac_dist_threshold, DEFAULT_RANSAC_DIST_THRESH);
	nh->param("ransac_line_dist_threshold", ransac_line_dist_threshold, DEFAULT_RANSAC_LINE_DIST_THRESH);
	nh->param("ransac_model_epsilon", ransac_model_epsilon, DEFAULT_RANSAC_MODEL_EPSILON);
	nh->param("concave_hull_alpha", concave_hull_alpha, DEFAULT_CONCAVE_HULL_ALPHA);

	nh->param("ortho_rad_thresh",ortho_rad_thresh, DEFAULT_ORTHO_RAD_THRESH);
	param_cos_ortho_tolerance = cos(std::abs(M_PI/2 - ortho_rad_thresh));

	//all publish options are set to false by default
	nh->param("publish_plane_pcl", param_publish_plane_pcl, false);
	nh->param("publish_hull_pcl", param_publish_hull_pcl, false );
	nh->param("publish_marker_rviz", param_publish_rviz_marker, false );
	nh->param("publish_outliers_pcl", param_publish_outliers_pcl, false);

	nh->param("max_height", max_height_plane, DEFAULT_MAX_HEIGHT);
	nh->param("min_height", min_height_plane, DEFAULT_MIN_HEIGHT);

	nh->param("incoming_pcl_sampling_period", param_interval, 1);

#ifdef DO_TIME_SPEC
	time_output.open("rect_detect_time_caract_output.csv");
	time_output << "init       ,get vert   ,convert msg,sel range  ,sac plane  ,union-find ,hull       ,borders    ,best rect  ,pub dim    ,end\n";
#endif

	geometry_model.setCosOrthoTolerance(param_cos_ortho_tolerance);

	ROS_INFO("starting ransac orthogonal plane detection");
	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh->subscribe ("input", 1, cloud_cb);

	// Create a ROS publisher for the output point cloud
	if(param_publish_plane_pcl)
		output_pcl_pub = nh->advertise<sensor_msgs::PointCloud2> ("output", 1);
	if(param_publish_hull_pcl)
		hull_pcl_pub = nh->advertise<sensor_msgs::PointCloud2> ("hull", 1);
	if(param_publish_outliers_pcl)
		output_pcl_outliers_pub = nh->advertise<sensor_msgs::PointCloud2> ("outliers",1);

	if(param_publish_rviz_marker)
		marker_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 1);

	dimensions_publisher = nh->advertise<geometry_msgs::Point> ("table_dimensions",1);

	// Spin
	ros::spin ();
#ifdef DO_TIME_SPEC
	time_output.close();
#endif
}

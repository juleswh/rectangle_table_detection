#include "ransac_plane_detection.h"


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
	pcl::IndicesPtr inliers(new std::vector<int>);

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
	//from these, create a line that will be used to get the height of the plane we find
	Eigen::ParametrizedLine<float,3> vertical_param_line_camera(Eigen::Vector3f(origin_world_camera[0],origin_world_camera[1],origin_world_camera[2]),Eigen::Vector3f(vert_camera[0],vert_camera[1],vert_camera[2]));

#ifdef DO_TIME_SPEC
	//get vertical
	time_output << ros::Time::now()-begin << ",";
#endif

	// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>());
	ROS_DEBUG_STREAM("input height " << input->height);
	pcl::fromROSMsg (*input, *cloud);

#ifdef DO_TIME_SPEC
	//convert msg
	time_output << ros::Time::now()-begin << ",";
#endif

	//select only points that are in the height range
	pcl::IndicesPtr in_range_filter_inliers;
	filter_out_of_range_points(cloud,in_range_filter_inliers,vertical_param_line_camera,min_height_plane,max_height_plane);

#ifdef DO_TIME_SPEC
	//select range
	time_output << ros::Time::now()-begin << ",";
#endif

	//search for the plane in the selected range
	ransac_plane_compute(cloud,vertical_param_line_camera,ransac_model_epsilon,ransac_dist_threshold,in_range_filter_inliers, inliers, coefficients, min_height_plane,max_height_plane);

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

	pcl::PointIndices::Ptr p_indices(new pcl::PointIndices);
	pcl::PointIndices::Ptr p_indices_filt(new pcl::PointIndices);
	p_indices->indices = *inliers;
	//ROS_DEBUG("input union-find %d points", p_indices->indices.size());

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

	std::vector<int> group_inliers;
	//compute the point cloud representing the plan
	pcl::ExtractIndices<pcl::PointXYZRGBA> filt_extract;
	filt_extract.setInputCloud (cloud);
	filt_extract.setIndices (p_indices);
	filt_extract.setNegative (false);
	filt_extract.filter (*plane_cloud);

	union_find(plane_cloud,plane_cloud->size()/200, group_inliers);
	p_indices_filt->indices = group_inliers;

#ifdef DO_TIME_SPEC
	//union-find
	time_output << ros::Time::now()-begin << ",";
#endif

	//statistical_filter(cloud,p_indices,p_indices->indices);
	//ROS_DEBUG("output union-find %d points", group_inliers.size());
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr hull = calc_concave_hull(plane_cloud, p_indices_filt, m_coefficients);
	/////////////////////////////////////////////////////////

#ifdef DO_TIME_SPEC
	//hull
	time_output << ros::Time::now()-begin << ",";
#endif

	/////////////////////////////////////////////////////////
	//	compute RANSAC lines of the hull ////////////////////
	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> lines_clouds;
	std::vector<pcl::IndicesPtr> lines_inliers;
	std::vector<Line_def*> borders_param_lines;
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
	std::vector<Eigen::VectorXf> lines_coeffs;
	std::vector<Vertex_def*> table_vertices;
	Rectangle table_rectangle;
	//a copy of the hull Pt Cld that will be modifided
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr borders_hull_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>(*hull));

	//finds all the lines and vertices
	size_t i=0;
	//while we have points in the point cloud (more than a certain percentage of the initial hull)
	while((borders_hull_cloud->size()>hull->size()*0.1) && (i < 10)){
		lines_clouds.push_back(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>));
		lines_inliers.push_back(pcl::IndicesPtr(new std::vector<int>));
		lines_coeffs.push_back(Eigen::VectorXf());

		find_line_ransac(borders_hull_cloud, ransac_dist_threshold, lines_inliers.back(),lines_coeffs.back());

		//check if we found a line or not, if not, exits loop
		if (lines_inliers.back()->size() == 0) {
			ROS_WARN("found only %d vertices with %d lines", table_vertices.size(),i);
			break;
		}

		//creates a parametrized line representing the border
		Eigen::Vector3f direction(lines_coeffs[i][3],lines_coeffs[i][4],lines_coeffs[i][5]);
		direction.normalize();

		borders_param_lines.push_back(new Line_def);
		borders_param_lines.back()->line=Eigen::ParametrizedLine<float,3>(Eigen::Vector3f(lines_coeffs[i][0],lines_coeffs[i][1],lines_coeffs[i][2]),direction);
		borders_param_lines.back()->marked=false;

		//finds vertices, defined as the intersection of 2 orthogonals edges
		for (size_t j=0 ; j < i ; j++){
			//TODO : correct bug when less than 4 vertices are found
			//ROS_DEBUG_STREAM("edges angle " << std::abs(borders_param_lines[j].direction().dot(borders_param_lines[i].direction())));
			if ( std::abs(borders_param_lines[j]->line.direction().dot(borders_param_lines[i]->line.direction())) < param_cos_ortho_tolerance ){
				//if they are orthogonals
				//we get a vertex of the table
				//we find their intersection = the projection of a point of the 1st line on the 2nd
				table_vertices.push_back(new Vertex_def);
				table_vertices.back()->vertex = borders_param_lines[j]->line.projection(borders_param_lines[i]->line.origin());
				table_vertices.back()->edges.push_back(borders_param_lines[i]);
				table_vertices.back()->edges.push_back(borders_param_lines[j]);
				borders_param_lines[i]->vertices.push_back(table_vertices.back());
				borders_param_lines[j]->vertices.push_back(table_vertices.back());
			}
		}

		//remove points of the line we just found to find another one in the next iteration
		extract.setInputCloud (borders_hull_cloud);
		extract.setIndices (lines_inliers.back());
		extract.setNegative (true);
		extract.filter (*borders_hull_cloud);

		i++;
	}

#ifdef DO_TIME_SPEC
	//borders detect
	time_output << ros::Time::now()-begin << ",";
#endif

	ROS_DEBUG("borders used %d",borders_param_lines.size());
	ROS_DEBUG("vertices found %d",table_vertices.size());
	std::vector<Rectangle> possible_rectangles;

	pcl::PointIndices::Ptr not_matched_points(new pcl::PointIndices);

	//if we have 4 vertices, it's ok, otherwise, we notify the user
	if(table_vertices.size() >= 4 ){
		//we search groups of connected vertices
		//i.e. (pieces of) rectangles
		for (int i=0; i<borders_param_lines.size(); i++){
			std::vector<Vertex_def*> connected_points;
			recursively_find_connected_vertices(connected_points,borders_param_lines[i]);
			//get the vertex that will be used as the frame of the plane 
			if (connected_points.size()==4){
				possible_rectangles.push_back(compute_rectangle(connected_points,vertical_param_line_camera.direction(),param_cos_ortho_tolerance));
			}else if ((connected_points.size() >4) && connected_points.size() <=6){
				std::vector<Vertex_def*> connected_points_sub(4);
				for(int i1=0; i1<connected_points.size();i1++){
					connected_points_sub[0]=connected_points[i1];
					for(int i2=0; i2<i1;i2++){
						connected_points_sub[1]=connected_points[i2];
						for(int i3=0; i3<i2;i3++){
							connected_points_sub[2]=connected_points[i3];
							for(int i4=0; i4<i3;i4++){
								connected_points_sub[3]=connected_points[i4];
								possible_rectangles.push_back(compute_rectangle(connected_points_sub,vertical_param_line_camera.direction(), param_cos_ortho_tolerance));
							}
						}
					}
				}
			}else if(connected_points.size()>6){
				ROS_WARN("too much vertices to compute all possible rectangles %d",connected_points.size());
			}else if (connected_points.size()>0){
				ROS_WARN("not enougth vertices to compute a rectangle %d", connected_points.size());
			}



		}

		ROS_DEBUG("possible rectangles = %d",possible_rectangles.size());

		//select the best rectangle
		//let's play a match
#ifdef _DEBUG_FUNCTIONS_
		float best_score;
		int best_index;
		int best_rectangle_index=select_best_matching_rectangle(possible_rectangles,plane_cloud,p_indices_filt,0.80,0.,plane_cloud->size()/10,best_score,best_index,not_matched_points);

		if (best_rectangle_index >=0){
			ROS_DEBUG("max score = %f, not matched : %d",best_score, not_matched_points->indices.size());
			table_rectangle = possible_rectangles[best_rectangle_index];
			plane_center_found = true;
		}else{
			ROS_WARN("did not find a good definition of the table");
			plane_center_found = false;
			//print a log
			std::ofstream debug_log_file;
			debug_log_file.open("debug_graph_file.tex",ios::app | ios::out);
			debug_log_file << "\\begin{figure}\n\\center\n";
			for (int i=0;i<borders_param_lines.size();i++){
				debug_log_file <<"\\begin{tikzpicture}\n";
				if(print_graph(borders_param_lines[i],0,debug_log_file) !=0 ){
					debug_log_file <<"\\end{tikzpicture}\noooooo\\\\ \n";
				}else{
					debug_log_file <<"\\end{tikzpicture}\n";
				}
			}
			debug_log_file << "\\caption{possible rectangles = " << possible_rectangles.size()
				<< "; edges = " << borders_param_lines.size() 
				<< "; vertices = " << table_vertices.size() 
				<< "best score = " << best_score
				<<", best dimensions " << possible_rectangles[best_index].vect_x.norm() 
				<< " " << possible_rectangles[best_index].vect_y.norm() 
				<< ", not matched " << not_matched_points->indices.size() 
				<< "}\n";
			debug_log_file << "\\end{figure}\n";
			debug_log_file.close();
		}

#else

		int best_rectangle_index=select_best_matching_rectangle(possible_rectangles,plane_cloud,p_indices_filt,0.80,0.,plane_cloud->size()/10);
		if (best_rectangle_index >=0){
			table_rectangle = possible_rectangles[best_rectangle_index];
			plane_center_found = true;
		}else{
			ROS_WARN("did not find a good definition of the table");
			plane_center_found = false;
		}
#endif

	}else{
		ROS_WARN_STREAM("table not defined, "<< table_vertices.size() << "vertices found");
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
		plane_tf = computeTransform (table_rectangle.point, table_rectangle.vect_x,table_rectangle.vect_y,param_cos_ortho_tolerance);

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
		//for (size_t i = 0; i < inliers->size (); ++i)
		//	cloud_inliers.insert(cloud_inliers.end(),cloud->points[(*inliers)[i]]);
		//for (size_t i = 0; i < p_indices_filt->indices.size (); ++i)
		for (size_t i = 0; i < not_matched_points->indices.size (); ++i)
			//cloud_inliers.insert(cloud_inliers.end(),plane_cloud->points[(p_indices_filt->indices)[i]]);
			cloud_inliers.insert(cloud_inliers.end(),plane_cloud->points[(not_matched_points->indices)[i]]);

		// convert and Publish the data
		pcl::toROSMsg(cloud_inliers,msg_cloud_inliers);
		//pcl::toROSMsg(*line1_cloud,msg_cloud_inliers);
		//modify the frame id
		msg_cloud_inliers.header.frame_id=input->header.frame_id;
		output_pcl_pub.publish (msg_cloud_inliers);
	}

	//publish the outliers point cloud
	if(param_publish_outliers_pcl){
		sensor_msgs::PointCloud2 msg_outliers;
		pcl::PointCloud<pcl::PointXYZRGBA> cloud_outliers;
		filt_extract.setInputCloud (cloud);
		filt_extract.setIndices (p_indices);
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

	for (int i=0; i<borders_param_lines.size();i++){
		delete borders_param_lines[i];
	}
	for (int i=0; i<table_vertices.size();i++){
		delete table_vertices[i];
	}

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
	nh->param("ransac_model_epsilon", ransac_model_epsilon, DEFAULT_RANSAC_MODEL_EPSILON);

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

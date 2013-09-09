#include "geometry_utilities.h"

#ifdef _DEBUG_FUNCTIONS_
int print_graph(Line_def* edge, int index,std::ostream& outstream){
	int r=0;
	if(edge==NULL) return r;
	if(!edge->marked) return r;
	edge->marked = false;
	r++;

	for (int i=0; i<edge->vertices.size(); i++){
		for (int j=0;j<i;j++){
			outstream << "\\draw (" << 4* edge->vertices[i]->vertex[0] << "," << (4* edge->vertices[i]->vertex[1] ) << ") -- (" <<4* edge->vertices[j]->vertex[0] << ","<< (4* edge->vertices[j]->vertex[1] ) << ");" << std::endl;
			//outstream << "\\draw (" << 4* edge->vertices[i]->vertex[0] << "," << (4* edge->vertices[i]->vertex[1] + 4* edge->vertices[i]->vertex[2]) << ") -- (" <<4* edge->vertices[j]->vertex[0] << ","<< (4* edge->vertices[j]->vertex[1] + 4* edge->vertices[j]->vertex[2]) << ");" << std::endl;
		}
		for (int e=0; e<edge->vertices[i]->edges.size(); e++){
			r+=print_graph(edge->vertices[i]->edges[e],0,outstream);
		}
	}
	return r;
}
#endif

inline float dot_product(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2){
	return (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2]);
}

int find_all_possible_rectangles(std::vector<boost::shared_ptr<Rectangle> >& possible_rectangles, const std::vector<Line_def*>& borders_param_lines, const Eigen::Vector3f& vertical, float param_cos_ortho_tolerance){
	//we search groups of connected vertices
	//i.e. (pieces of) rectangles
	for (int i=0; i<borders_param_lines.size(); i++){
		std::vector<Vertex_def*> connected_points;
		recursively_find_connected_vertices(connected_points,borders_param_lines[i]);
		if (connected_points.size()==4){
			possible_rectangles.push_back(compute_rectangle(connected_points,vertical,param_cos_ortho_tolerance));
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
							possible_rectangles.push_back(compute_rectangle(connected_points_sub,vertical, param_cos_ortho_tolerance));
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

	return possible_rectangles.size();
}

void recursively_find_rectangles(std::vector<boost::shared_ptr<Rectangles> >& rectangles, std::vector<Vertex_def*>& vertices, const Line_def* prev_edge){
	if (vertices.size() == 5){
		if (vertices.back() == vertices.front()){
			rectangles.push_back(compute_rectangle(vertices.begin(),vertices.begin()+4,vertical, param_cos_ortho_tolerance));
		}
	}else{
		for (int i=0; i< vertices.back()->edges.size()){
			if ( vertices.back()->edges[i] != prev_edge ){
				for (int p=0; p<vertices.back()->edges[i]->vertices.size();p++){
					if (vertices.back()->edges[i]->vertices[p] != vertices.front()){
						vertices.push_back(vertices.back()->edges[i]->vertices[p]);
						recursively_find_rectangles(rectangles, vertices,vertices.back());
					}
				}
			}
		}
	}
	vertices.pop_back();
}

void recursively_find_connected_vertices(std::vector<Vertex_def*>& vertices, Line_def* edge){
	if(edge==NULL) return;
	//if already visited, we're done with this one
	if(edge->marked) return;
	//we mark this one
	edge->marked = true;
	//if have less than 2 vertices it does not connect any points, ignore this
	if(edge->vertices.size() < 2) return;

	//for each vertex the edge have, we add it to the vertex list and recursively
	//search on its edges
	for (int i=0; i < edge->vertices.size(); i++){
		if ( std::find(vertices.begin(),vertices.end(),edge->vertices[i])==vertices.end()){
			vertices.push_back(edge->vertices[i]);
			for (int j=0; j<edge->vertices[i]->edges.size(); j++){
				recursively_find_connected_vertices(vertices, edge->vertices[i]->edges[j]);
			}
		}
	}
	return;
}

bool point_is_in_rectangle(const pcl::PointXYZRGBA point, const Rectangle& rect, float relative_thresh=0.01){
	bool rval;
	Eigen::Vector3f p(point.x, point.y,point.z);
	p-=rect.point;
	float projx = dot_product(p,rect.vect_x);
	float projy = dot_product(p,rect.vect_y);
	float projz = dot_product(p,rect.vect_x.cross(rect.vect_y));

	rval = true;
	if	(!(projx <= (1+relative_thresh) * rect.vect_x.norm())){
	// ROS_DEBUG("x+ : %f",(projx - (1+relative_thresh) * rect.vect_x.norm()));
	 rval=false;
	}
	if (!(projy <= (1+relative_thresh) * rect.vect_y.norm())){
		//ROS_DEBUG("y+ : %f",(projy - (1+relative_thresh) * rect.vect_y.norm()));
		rval=false;
	}
	if (!(projx >= - relative_thresh * rect.vect_x.norm())){
		//ROS_DEBUG("x- : %f",(projx + relative_thresh * rect.vect_x.norm()));
		rval=false;
	}
 	if (!(projy >=  - relative_thresh * rect.vect_y.norm())){
		//ROS_DEBUG("y- : %f",(projy + relative_thresh * rect.vect_y.norm()));
		rval=false;
	}
	//if(!rval)
	//	ROS_DEBUG("proj x %f y %f, p.norm %f, calc norm %f",projx,projy, p.norm(), sqrt(pow(projx,2)+pow(projy,2)+pow(projz,2)));
	return rval;
}

/**
 * select the rectangle that best describes the plan in the point cloud pc_plan
 * select randomly n_samples points in the input pc_plan
 * and check for each rectangle if it contains this point
 * Each rectangle have a score between 0 and 1
 * If the best rectangle have a score > required_score
 * and a <lead_score> points lead on the 2nd best, 
 * we return it's index
 * returns -1 if no good rectangle found
 **/
int select_best_matching_rectangle(const std::vector<Rectangle>& possible_rectangles,
		pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr pc_plan,
	 	pcl::PointIndices::ConstPtr indices,
	 	float required_score, float lead_score,
	 	int n_samples,
	 	float& best_score, int& best_index,
	 	pcl::PointIndices::Ptr not_matched_points){
	
	std::vector<float> scores(possible_rectangles.size());
	std::vector<pcl::PointIndices::Ptr> not_matched;
	if (not_matched_points){
		not_matched.resize(possible_rectangles.size(),pcl::PointIndices::Ptr(new pcl::PointIndices));
	}
	int i = 0;
	int r_val;
	int rand_index;
	float max_score=0,second_score=-1;
	int max_score_index=-1,second_score_index=-2;
	while (i<n_samples){
		rand_index = rand() % indices->indices.size();

		for (int rect_i=0; rect_i<possible_rectangles.size(); rect_i++){
			if (point_is_in_rectangle(pc_plan->at(indices->indices[rand_index]), possible_rectangles[rect_i])){
				scores[rect_i]++;
			}else if(not_matched_points){
				not_matched[rect_i]->indices.push_back(indices->indices[rand_index]);
			}
		}
		i++;
	}

	//search for max and 2nd max scores
	for(int i=0; i<scores.size(); i++){
		//put score in [0,1]
		scores[i]/=n_samples;
		if(scores[i]>max_score){
			second_score=max_score;
			second_score_index=max_score_index;
			max_score = scores[i];
			max_score_index = i;
		}
	}

	//make sure the best mathcing rectangle have a very high score
	//and is much better than the second
	//TODO : implement "prolongations" if no good rectangle is found : use more samples
	if ((max_score > required_score) && (max_score - second_score > lead_score)) {
		r_val = max_score_index;
	}else{
		r_val = -1;
	}
	best_score = max_score;
	best_index = max_score_index;
	if(not_matched_points){
		*not_matched_points = *not_matched[max_score_index];
		ROS_WARN("not_matched size = %d, return %d",not_matched[max_score_index]->indices.size(),not_matched_points->indices.size());
	}
	return r_val;
}

int select_best_matching_rectangle(const std::vector<Rectangle>& possible_rectangles,
		pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr pc_plan,
	 	pcl::PointIndices::ConstPtr indices,
	 	float required_score, float lead_score,
	 	int n_samples){
	float best_score;
	int best_score_index;
	pcl::PointIndices::Ptr not_matched;
	return select_best_matching_rectangle(possible_rectangles, pc_plan, indices,
			required_score, lead_score,
			n_samples,
			best_score,best_score_index, not_matched);
}





/**
	* returns a rectangle from the given vertices
	* the returned rectangle is such that all points in it can
	* be defined as a*vect_x + b*vect_y
	* with a and b >=0
	* vect_x and vect_y are choosen so that vect_x x vect_y have (approx) the same direction 
	* than the given vertical
	**/
boost::shared_ptr<Rectangle> compute_rectangle(std::vector<Vertex_def*>& vertices, Eigen::Vector3f vertical, double param_cos_ortho_tolerance){
	bool ok = true;
	boost::shared_ptr<Rectangle> p_rect;
	Rectangle rect;
	std::vector<Eigen::Vector3f> vectors;

	if (vertices.size() != 4 ){
		ROS_WARN("they want me to compute a rectangle with %d vertices?!?",vertices.size());
		return p_rect;
	}

	rect.point = vertices[0]->vertex;

	//compute vectors from main vertex to other vertices
	for (size_t i=1 ; i < vertices.size() ; i++){
		vectors.push_back(vertices[i]->vertex - rect.point);
	}

	//from these vectors, find the two that defines the rectangle
	for (size_t i=0 ; i < vectors.size() ; i++){
		for (size_t j=0 ; j < i ; j++){
			double cos_angle = vectors[i].dot(vectors[j]) / (vectors[i].norm() * vectors[j].norm());
			//if the cos of the angle between i and j is approx = 0, (i.e. angle ~ 90 degrees)
			if (std::abs(cos_angle) < param_cos_ortho_tolerance){
				float dir = vectors[i].cross(vectors[j]).dot(vertical);
				//if the dot product is >0, the angle from i to j is >0, so i is x and j is y
				//inverted otherwise
				if (dir > 0){
					rect.vect_x = vectors[i];
					rect.vect_y = vectors[j];
				}else{
					rect.vect_x = vectors[j];
					rect.vect_y = vectors[i];
				}
			}else if (std::abs(vectors[i].norm() - vectors[j].norm())/(vectors[i].norm() + vectors[j].norm()) > 0.1){
				//if two borders that are not orthogonal have a difference of distance > 10%, there is an error
				ok=false;
			}

		}
	}

	if(ok){
		//if there is no error, we return a ptr to the rectangle, otherwise an empty ptr
		p_rect = boost::shared_ptr<Rectangle>(rect);
	}

	return p_rect;
}

/**
	* compute a transform so that the child frame have given x and y axis
	* (nota : axis CAN be not normalized)
	**/
tf::Transform computeTransform(Eigen::Vector3f origin, Eigen::Vector3f x_axis,Eigen::Vector3f y_axis,double param_cos_ortho_tolerance)
{
	tf::Transform transform;
	Eigen::Quaternionf rotation1_e, rotation2_e;

	if (std::abs(x_axis.dot(y_axis)/(x_axis.norm()*y_axis.norm())) > param_cos_ortho_tolerance){
		ROS_WARN("given axes are not orthogonals for transfrom computation, aborting\n(cos value : %f)",x_axis.dot(y_axis)/(x_axis.norm()*y_axis.norm()));

	}else{

		rotation1_e.setFromTwoVectors(Eigen::Vector3f(1,0,0),x_axis);
		rotation2_e.setFromTwoVectors(rotation1_e * Eigen::Vector3f(0,1,0) , y_axis);

		rotation2_e *= rotation1_e;
		tf::Quaternion rotation_tf(rotation2_e.x(), rotation2_e.y(), rotation2_e.z(), rotation2_e.w());

		//set the origin to the given point
		transform.setOrigin( tf::Vector3(origin[0], origin[1], origin[2]) );
		transform.setRotation( rotation_tf );
	}
	return transform;
}

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

#include "tableDetectionGeometricModel.h"

typedef tableDetectionGeometricModel::Rectangle Rectangle;
typedef tableDetectionGeometricModel::Vertex_def Vertex_def;
typedef tableDetectionGeometricModel::Line_def Line_def;

tableDetectionGeometricModel::tableDetectionGeometricModel(Eigen::Vector3f origin, Eigen::Vector3f vertical, double cos_ortho_tolerance){
	this->init(cos_ortho_tolerance);
	this->setVerticalLine(origin,vertical);
}

tableDetectionGeometricModel::tableDetectionGeometricModel(double cos_ortho_tolerance){
	this->init(cos_ortho_tolerance);
}

void tableDetectionGeometricModel::init(double cos_ortho_tolerance){
	Eigen::Vector3f vzero(0,0,0);
	this->_cos_ortho_tolerance = cos_ortho_tolerance;
	this->_previous_best_rectangle = boost::shared_ptr<Rectangle>(new Rectangle);
	this->_previous_best_rectangle->vect_x = this->_previous_best_rectangle->vect_y = this->_previous_best_rectangle->point = vzero;
}

tableDetectionGeometricModel::~tableDetectionGeometricModel(){
	for (int i=0; i<this->borders_.size();i++){
		delete this->borders_[i];
	}
	for (int i=0; i<this->vertices_.size();i++){
		delete this->vertices_[i];
	}
}

void tableDetectionGeometricModel::clear(){
	for (int i=0; i<this->borders_.size();i++){
		delete this->borders_[i];
	}
	this->borders_.clear();
	for (int i=0; i<this->vertices_.size();i++){
		delete this->vertices_[i];
	}
	this->vertices_.clear();
	this->possible_rectangles_.clear();
}


bool tableDetectionGeometricModel::addVertexFromEdges(int i,int j){
	bool rval;
	if ((i<this->bordersCount() ) && ( j<this->bordersCount() )){
		this->vertices_.push_back(new Vertex_def);
		this->vertices_.back()->vertex = this->borders_[j]->line.projection(this->borders_[i]->line.origin());
		this->vertices_.back()->edges.push_back(this->borders_[i]);
		this->vertices_.back()->edges.push_back(this->borders_[j]);
		this->borders_[i]->vertices.push_back(this->vertices_.back());
		this->borders_[j]->vertices.push_back(this->vertices_.back());
		rval=true;
	}else
		rval=false;

	return rval;
}


void tableDetectionGeometricModel::addBorder(const Eigen::VectorXf& coeffs){
	Eigen::Vector3f direction(coeffs[3],coeffs[4],coeffs[5]);
	direction.normalize();

	this->borders_.push_back(new Line_def);
	this->borders_.back()->line=Eigen::ParametrizedLine<float,3>(Eigen::Vector3f(coeffs[0],coeffs[1],coeffs[2]),direction);
	this->borders_.back()->marked=false;
}

bool tableDetectionGeometricModel::areBordersOrthogonal(int i, int j){
 return (std::abs(this->borders_[j]->line.direction().dot(this->borders_[i]->line.direction())) < this->_cos_ortho_tolerance );
}

bool tableDetectionGeometricModel::addPossibleRectangle(const std::vector<int>& vertices_indices){
	boost::shared_ptr<Rectangle> rect;
	rect = this->compute_rectangle(vertices_indices);
	if(rect){
		this->possible_rectangles_.push_back(rect);
		return true;
	}else{
		return false;
	}
}

bool tableDetectionGeometricModel::addPossibleRectangle(const std::vector<Vertex_def*>::iterator& from_vertex, const std::vector<Vertex_def*>::iterator& to_vertex){
	boost::shared_ptr<Rectangle> rect;
	rect = this->compute_rectangle(from_vertex,to_vertex);
	if(rect){
		this->possible_rectangles_.push_back(rect);
		return true;
	}else{
		return false;
	}
}

bool tableDetectionGeometricModel::addPossibleRectangle(const std::vector<Vertex_def*>& vertices){
	boost::shared_ptr<Rectangle> rect;
	rect = this->compute_rectangle(vertices);
	if(rect){
		this->possible_rectangles_.push_back(rect);
		return true;
	}else{
		return false;
	}
}


void tableDetectionGeometricModel::setVerticalLine(const Eigen::Vector3f& origin, const Eigen::Vector3f& direction){
	this->setVerticalLine(Eigen::ParametrizedLine<float,3>(origin,direction));
}
void tableDetectionGeometricModel::setVerticalLine(const Eigen::ParametrizedLine<float,3>& vertical_line){
	this->_vertical_line = vertical_line;
}

Eigen::ParametrizedLine<float,3> tableDetectionGeometricModel::getVerticalLine(){
	return this->_vertical_line;
}
Eigen::Vector3f tableDetectionGeometricModel::getVerticalOrigin(){
	return this->_vertical_line.origin();
}
Eigen::Vector3f tableDetectionGeometricModel::getVerticalDirection(){
	return this->_vertical_line.direction();
}
int tableDetectionGeometricModel::verticesCount(){
	return this->vertices_.size();
}
int tableDetectionGeometricModel::bordersCount(){
	return this->borders_.size();
}
int tableDetectionGeometricModel::possibleRectanglesCount(){
	return this->possible_rectangles_.size();
}
void tableDetectionGeometricModel::setCosOrthoTolerance(double tol){
	this->_cos_ortho_tolerance = tol;
}
double tableDetectionGeometricModel::getCosOrthoTolerance(){
	return this->_cos_ortho_tolerance;
}




#ifdef _DEBUG_FUNCTIONS_
int tableDetectionGeometricModel::print_graph(Line_def* edge, int index,std::ostream& outstream){
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

inline float tableDetectionGeometricModel::dot_product(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2){
	return (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2]);
}

int tableDetectionGeometricModel::find_all_possible_rectangles(){
	//we search groups of connected vertices
	//i.e. (pieces of) rectangles
	/**
	for (int i=0; i<this->borders_.size(); i++){
		std::vector<Vertex_def*> connected_points;
		recursively_find_connected_vertices(connected_points,this->borders_[i]);
		if (connected_points.size()==4){
			this->addPossibleRectangle(connected_points);
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
							this->addPossibleRectangle(connected_points_sub);
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
	*/

	for (std::vector<Vertex_def*>::const_iterator it=this->vertices_.begin(); it!=this->vertices_.end();++it){
		this->recursively_find_rectangles_from(*it);

	}

	for (int i=0;i<this->_verticesLoopsMngr._loops_collection.size();i++){
		this->addPossibleRectangle(this->_verticesLoopsMngr._loops_collection[i]);
		ROS_DEBUG("add rect x=%f y=%f", this->possible_rectangles_.back()->vect_x.norm(), this->possible_rectangles_.back()->vect_y.norm() );
	}
	this->_verticesLoopsMngr._loops_collection.clear();

	return this->possible_rectangles_.size();
}

void tableDetectionGeometricModel::recursively_find_rectangles_from(Vertex_def* vertex){
	std::vector<Vertex_def*> vertices;
	vertices.push_back(vertex);
	this->recursively_find_rectangles(vertices,NULL);
}
	

void tableDetectionGeometricModel::recursively_find_rectangles(std::vector<Vertex_def*>& vertices, const Line_def* prev_edge){
	//simple case : we have 5 vertices, simply determine if the first = the last
	int depth = vertices.size()-1;
	if (depth == 4){
		if (vertices[depth]->vertex == vertices.front()->vertex){
			//the rectangle formed by this 4 vertices is a rectangle :
			this->_verticesLoopsMngr.addLoop(std::vector<Vertex_def*>(vertices.begin(),vertices.begin()+4));
		}
	}else{
		//otherwise, recurse in the edges/vertices tree
		for (int i=0; i< vertices[depth]->edges.size();i++){
			if (prev_edge!=vertices[depth]->edges[i]){
				for (int p=0; p<vertices[depth]->edges[i]->vertices.size();p++){
					if (vertices[depth]->edges[i]->vertices[p] != vertices[depth]){
						vertices.push_back(vertices[depth]->edges[i]->vertices[p]);
						recursively_find_rectangles(vertices,vertices[depth]->edges[i]);
					}
				}
			}
		}
	}
	vertices.pop_back();
}

void tableDetectionGeometricModel::recursively_find_connected_vertices(std::vector<Vertex_def*>& vertices, Line_def* edge){
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

bool tableDetectionGeometricModel::point_is_in_rectangle(const pcl::PointXYZRGBA point, const boost::shared_ptr<Rectangle> rect, float relative_thresh){
	bool rval;
	Eigen::Vector3f p(point.x, point.y,point.z);
	p-=rect->point;
	float projx = dot_product(p,rect->vect_x);
	float projy = dot_product(p,rect->vect_y);
	float projz = dot_product(p,rect->vect_x.cross(rect->vect_y));

	rval = true;
	if	(!(projx <= (1+relative_thresh) * rect->vect_x.norm())){
	// ROS_DEBUG("x+ : %f",(projx - (1+relative_thresh) * rect->vect_x.norm()));
	 rval=false;
	}
	if (!(projy <= (1+relative_thresh) * rect->vect_y.norm())){
		//ROS_DEBUG("y+ : %f",(projy - (1+relative_thresh) * rect->vect_y.norm()));
		rval=false;
	}
	if (!(projx >= - relative_thresh * rect->vect_x.norm())){
		//ROS_DEBUG("x- : %f",(projx + relative_thresh * rect->vect_x.norm()));
		rval=false;
	}
 	if (!(projy >=  - relative_thresh * rect->vect_y.norm())){
		//ROS_DEBUG("y- : %f",(projy + relative_thresh * rect->vect_y.norm()));
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
int tableDetectionGeometricModel::select_best_matching_rectangle(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr pc_plan,
	 	pcl::PointIndices::ConstPtr indices,
	 	float required_score, float lead_score,
	 	int n_samples,
	 	float& best_score, int& best_index,
	 	pcl::PointIndices::Ptr not_matched_points){

	if(this->possibleRectanglesCount()<=0)
		return -1;
	
	std::vector<float> scores(this->possible_rectangles_.size());
	std::vector<pcl::PointIndices::Ptr> not_matched;
	if (not_matched_points){
		not_matched.resize(this->possible_rectangles_.size(),pcl::PointIndices::Ptr(new pcl::PointIndices));
	}
	int i = 0;
	int r_val;
	int rand_index;
	float max_score=-1,second_score=-2;
	int max_score_index=-1,second_score_index=-2;
	while (i<n_samples){
		rand_index = rand() % indices->indices.size();

		for (int rect_i=0; rect_i<this->possible_rectangles_.size(); rect_i++){
			if (this->point_is_in_rectangle(pc_plan->at(indices->indices[rand_index]), this->possible_rectangles_[rect_i])){
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
		this->_previous_best_rectangle = this->possible_rectangles_[max_score_index];
	}else{
		r_val = -1;
	}
	best_score = max_score;
	best_index = max_score_index;
	ROS_DEBUG_STREAM("best score = "<<best_score << " index= "<<best_index);
	if(not_matched_points){
		not_matched_points = pcl::PointIndices::Ptr(not_matched[max_score_index]);
		ROS_WARN("not_matched size = %d, return %d",not_matched[max_score_index]->indices.size(),not_matched_points->indices.size());
	}
	return r_val;
}

int tableDetectionGeometricModel::select_best_matching_rectangle(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr pc_plan,
	 	pcl::PointIndices::ConstPtr indices,
	 	float required_score, float lead_score,
	 	int n_samples){
	float best_score;
	int best_score_index;
	pcl::PointIndices::Ptr not_matched;
	return select_best_matching_rectangle(pc_plan, indices,
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
boost::shared_ptr<Rectangle> tableDetectionGeometricModel::compute_rectangle(std::vector<Vertex_def*>::const_iterator from_vertex, std::vector<Vertex_def*>::const_iterator to_vertex){
	bool ok = true;
	boost::shared_ptr<Rectangle> p_rect(new Rectangle);
	std::vector<Eigen::Vector3f> vectors;
	float min_dist = FLT_MAX;
	std::vector<Vertex_def*>::const_iterator min_vertex;

	if ((to_vertex - from_vertex) != 4 ){
		ROS_WARN("they want me to compute a rectangle with %d vertices?!?",(to_vertex - from_vertex));
		p_rect.reset();
		return p_rect;
	}

	for (std::vector<Vertex_def*>::const_iterator vertex = from_vertex; vertex!=to_vertex; vertex++ ){
		Eigen::Vector3f diff = (*vertex)->vertex - this->_previous_best_rectangle->point;
		if( diff.norm() < min_dist ){
			min_dist = diff.norm();
			min_vertex = vertex;
		}
	}

	p_rect->point = (*min_vertex)->vertex;

	//compute vectors from main vertex to other vertices
	for (std::vector<Vertex_def*>::const_iterator vertex = from_vertex; vertex!=to_vertex; vertex++ ){
		if(min_vertex!=vertex)
			vectors.push_back((*vertex)->vertex - p_rect->point);
	}

	//from these vectors, find the two that defines the rectangle
	for (size_t i=0 ; i < vectors.size() ; i++){
		for (size_t j=0 ; j < i ; j++){
			double cos_angle = vectors[i].dot(vectors[j]) / (vectors[i].norm() * vectors[j].norm());
			//if angle ~ 90 degrees
			if (std::abs(cos_angle) < this->_cos_ortho_tolerance){
				float dir = vectors[i].cross(vectors[j]).dot(this->getVerticalDirection());
				//if the dot product is >0, the angle from i to j is >0, so i is x and j is y
				//inverted otherwise
				if (dir > 0){
					p_rect->vect_x = vectors[i];
					p_rect->vect_y = vectors[j];
				}else{
					p_rect->vect_x = vectors[j];
					p_rect->vect_y = vectors[i];
				}
			}
		}
	}

	if(!ok){
		//if there is no error, we return a ptr to the rectangle, otherwise an empty ptr
		p_rect.reset();
	}
	return p_rect;
}

boost::shared_ptr<Rectangle> tableDetectionGeometricModel::compute_rectangle(const std::vector<int>& vertices_indices){
	std::vector<Vertex_def*> vertices;
	for (std::vector<int>::const_iterator it=vertices_indices.begin(); it !=vertices_indices.end(); it++){
		vertices.push_back(this->vertices_[*it]);
	}
	return this->compute_rectangle(vertices);
}

boost::shared_ptr<Rectangle> tableDetectionGeometricModel::compute_rectangle(const std::vector<Vertex_def*>& vertices){
	return this->compute_rectangle(vertices.begin(),vertices.end());
}

/**
	* compute a transform so that the child frame have given x and y axis
	* (nota : axis CAN be not normalized)
	**/
tf::Transform tableDetectionGeometricModel::computeTransform(const Eigen::Vector3f& origin, const Eigen::Vector3f& x_axis,const Eigen::Vector3f& y_axis)
{
	tf::Transform transform;
	Eigen::Quaternionf rotation1_e, rotation2_e;

	if (std::abs(x_axis.dot(y_axis)/(x_axis.norm()*y_axis.norm())) > this->_cos_ortho_tolerance){
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


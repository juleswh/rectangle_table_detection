#include "group_matrix.h"

void makeSet(union_find_node* node){
	node->parent = node;
	node->rank=0;
	node->children_n = 1;
}

void union_nodes(union_find_node* x, union_find_node* y){
	union_find_node* xRoot = find(x);
	union_find_node* yRoot = find(y);
	if (xRoot != yRoot){
		if (xRoot->rank < yRoot->rank){
			xRoot->parent = yRoot;
			yRoot->children_n += xRoot->children_n;
		}else if (xRoot->rank > yRoot->rank){
			yRoot->parent = xRoot;
			xRoot->children_n += yRoot->children_n;
		}else{
			yRoot->parent = xRoot;
			xRoot->rank++; 
			xRoot->children_n += yRoot->children_n;
		}
	}
}

union_find_node* find(union_find_node* x){
	if(x == NULL) return NULL;
	if ( x->parent != x )
		x->parent = find(x->parent);
	return x->parent;
}


//updates a forest 
//the forest may contain all and only roots
void addToForest(std::vector<union_find_node*>& forest,union_find_node* node){
	int i=0;
	union_find_node* root = find (node);
	bool found=false;
	for (std::vector<union_find_node*>::iterator it=forest.begin(); it!=forest.end();){
		//find the root
		if (*it == root){
			found=true;
		}
		//remove node that are not roots
		if(find(*it)!=*it){
			it=forest.erase(it);
		}else{
			++it;
		}
	}
	//if not found, we add the root
	if(!found){
		forest.push_back(root);
	}
}


void union_find(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, int K, std::vector<int>& inliers){

	if (cloud->size()==0){
		return;
	}
	if(K==0){
		ROS_WARN("parameter K for K nearest neighbor is 0, aborting");
		return;
	}
	std::vector<union_find_node*> forest;
	std::vector<union_find_node*> model(cloud->size(),NULL);//init to NULL
	std::vector<int> knnindices(K); //stores indices of the last K nearest neighbors search
	std::vector<float> knnSqDistances(K); //sotres squared distance of the last KNN search
	int knncount=0;

	//KDTree
	pcl::KdTreeFLANN<pcl::PointXYZRGBA> kdtree;

	kdtree.setInputCloud (cloud);

	for (int i=0;i<cloud->size();i++){
		//if point is not yet visited
		if(model[i]==NULL){
			//find K nearest neighbors of the curent point
			knncount = kdtree.nearestKSearch(cloud->at(i),K,knnindices,knnSqDistances);
			//union them to the curent point
			for (int result_i=0;result_i<knncount;result_i++){
				//create the point in the model if it do not exists
				if (model[knnindices[result_i]]==NULL){
					model[knnindices[result_i]] = new union_find_node;
					makeSet(model[knnindices[result_i]]);
				}
				//actualy do the union
				union_nodes(model[i],model[knnindices[result_i]]);
			}
			//add the root to the forest (check if do not already exists)
			addToForest(forest,find(model[i]));
		}
	}

	//now we have to find the biggest tree
	if(forest.size()!=0){
		size_t max_size=0, biggest_tree_i=0;
		for (size_t i=0;i<forest.size();i++){
			if( forest[i]->children_n > max_size){
				max_size = forest[i]->children_n;
				biggest_tree_i = i;
			}
		}

		union_find_node* the_root = forest[biggest_tree_i];

		//we add to the inliers all the points that have the_root as root
		for (int i=0; i < cloud->size(); i++){
			if(find(model[i])==the_root){
				inliers.push_back(i);
			}
		}
		for (int i=0;i<model.size();i++)
			delete model[i];
	}
}


#ifdef _GROUP_MATRIX_MAIN_TEST_
int main(int argc,char** argv){
	// Initialize ROS
	ros::init (argc, argv, "test_unionfind");
	ros::NodeHandle* nh = new ros::NodeHandle("~");
	ros::Publisher pub = nh->advertise<sensor_msgs::PointCloud2> ("inliers", 1);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_in( new pcl::PointCloud<pcl::PointXYZRGBA>(10,10));
	std::vector<int> inliers;

	for(int col = 0 ; col < 10 ; col ++){
		for (int row =0 ; row <10; row ++){
			cloud_in->at(col,row).x = col;
			cloud_in->at(col,row).y = row;
		}
	}


	//first group
	cloud_in->at(2,1).a = 1;
	cloud_in->at(2,2).a = 1;
	cloud_in->at(2,0).a = 1;
	cloud_in->at(1,1).a = 1;
	cloud_in->at(3,1).a = 1;

	//second group
	cloud_in->at(5,4).a = 1;
	cloud_in->at(6,4).a = 1;
	cloud_in->at(6,5).a = 1;

	//third group
	cloud_in->at(4,7).a = 1;

	cloud_in->at(5,6).a = 1;
	//cloud_in->at(5,7).a = 1;
	cloud_in->at(6,6).a = 1;

	union_find(cloud_in,inliers);


	sensor_msgs::PointCloud2 msg_cloud_inliers;
	pcl::PointCloud<pcl::PointXYZRGBA> cloud_inliers;
	//for (size_t i = 0; i < inliers->size (); ++i)
	//	cloud_inliers.insert(cloud_inliers.end(),cloud->points[(*inliers)[i]]);
	for (size_t i = 0; i < inliers.size (); ++i)
		cloud_inliers.insert(cloud_inliers.end(),cloud_in->points[inliers[i]]);

	// convert and Publish the data
	//pcl::toROSMsg(cloud_inliers,msg_cloud_inliers);
	pcl::toROSMsg(cloud_inliers,msg_cloud_inliers);
	//pcl::toROSMsg(*line1_cloud,msg_cloud_inliers);
	//modify the frame id
	msg_cloud_inliers.header.frame_id="/world";
	while(nh->ok()){
	pub.publish (msg_cloud_inliers);
	}

	return 0;
}
#endif

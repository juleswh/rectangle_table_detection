/**
 * a clustering method using union-find algorithm to look for groups of points.
 * \file group_matrix.h
 *
 * Uses a kdtree to find k-nearest neighbors over a point cloud.
 * Usefull to find a group of points in a point cloud
 **/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/matrix.hpp>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>

//a debug utility
#define _PRINT_HERE_(s) ROS_DEBUG_STREAM("in " << __FILE__ << " line " << __LINE__ << "\t-\t" << s)

/**
 * the node definition used by the algorithm
 **/
struct union_find_node{
	union_find_node* parent; /**< the parent of this node**/
	size_t rank; /**< the rank of the node, for optimized union**/
	size_t children_n; /**< the number of children, to ease the search of biggest group**/
};

/**
 * Creates an empty node.
 **/
void makeSet(union_find_node* node);

/**
 * Unions two nodes so the biggest one is the parent of the other
 **/
void union_nodes(union_find_node* x, union_find_node* y);

/**
 * find the root of the tree where this node resides
 **/
union_find_node* find(union_find_node* x);

/**
	* returns the biggest group of points.
	* \param[in] cloud the cloud to compute
	* \param[out] inliers the inliers of cloud belonging to the biggest group
	*
	* Uses a union find algorithm
	* and uses K nearest neighbors pcl algorithm
	* the K nearest neighbors of a point are added 
	* to the same group than the point
	**/
void union_find(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, int K, std::vector<int>& inliers);

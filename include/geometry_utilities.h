/**
 * Data structures and functions in relation to the geometry of a table.
 * \file geometry_utilities.h
 *
 * These function mainly work over Eigen and are used to compute a model of the table
 **/
#ifndef _GEOMETRY_UTILITIES_H_
#define _GEOMETRY_UTILITIES_H_

#include <cmath>
#include <iostream>
#include <ostream>
#include <fstream>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// PCL specific includes
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>


//a debug utility
#define _PRINT_HERE_(s) ROS_DEBUG_STREAM("in " << __FILE__ << " line " << __LINE__ << "\t-\t" << s)


class geometryModel {

	public:

		struct Line_def;

		/**
		 * a vertex representation with a list of lines passing through this point.
		 * This structure represent a node of a graph formed by vertices and lines
		 * \see recursively_find_connected_vertices()
		 **/
		struct Vertex_def{
			std::vector<Line_def*> edges; /**< edges passing by this point (usually, the vertex is defined as the intersection of these edges)**/
			Eigen::Vector3f vertex; /**< the position in space of this vertex**/
		};

		/**
		 * a line representation
		 * \see Vertex_def
		 * \see recursively_find_connected_vertices()
		 **/
		struct Line_def{
			Eigen::ParametrizedLine<float,3> line; /**< the line definition as an Eigen object**/
			std::vector<Vertex_def*> vertices; /**< the vertices that are on this line**/
			bool marked; /**< a marker for the exploration of the graph**/
		};


		/**
		 * defines an oriented rectangle in a 3D space.
		 * The vector \f$ \overrightarrow{x} x \overrightarrow{y} \f$ indicates the orientation of the rectangle.
		 * So this 3 vectors and the point would form an orthogonal repair in the space.
		 **/
		struct Rectangle{
			Eigen::Vector3f vect_x; /**< the vector defining the 1st edge**/
			Eigen::Vector3f vect_y; /**< the vector defining the 2nd edge, orthogonal to the 1st**/
			Eigen::Vector3f point; /**< a point that is a vertex of the rectangle**/
		};

		geometryModel(Eigen::Vector3f origin, Eigen::Vector3f vertical, double param_cos_ortho_tolerance);
		virtual ~geometryModel();
			

	private:
		Eigen::ParametrizedLine<float, 3> _vertical_line;

		double cos_ortho_tolerance;

	public:
		std::vector<Line_def*> borders;
		std::vector<Vertex_def*> vertices;
		std::vector<boost::shared_ptr<Rectangle> > possible_rectangles;

	public:

		bool areBorderOrthogonals(int i,int j);
		bool addVertexFromEdges(int i,int j);
		void addBorder(const Eigen::VectorXf& coeffs);

#ifdef _DEBUG_FUNCTIONS_
		//TODO remove debug function
		int print_graph(Line_def* edge, int index, std::ostream& outstream);
#endif

		/** A redefinition of the dot product.
		 * \param[in] v1 first vector
		 * \param[in] v2 2nd vector
		 * \return the dot product \f$ \overrightarrow{v_1} \cdot \overrightarrow{v_2} \f$
		 *
		 * Apparently, the Eigen dot product is not efficient
		 */
		inline float dot_product(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);

		/**
		 * recursively explore the vertices of an edge an add connected points to vertices.
		 * \param[out] vertices vector to which connected vertices will be appended
		 * \param[in,out] edge edge where exploration starts
		 *
		 * Run over the graph formed by Line_def and Vertex_def and returns a list of pointers to vertices
		 * that are all connected by edges. This list can represent one or more overlapping rectangles, assuming
		 * that a vertex is defined as the intersection of two orthogonals lines.
		 **/
		void recursively_find_connected_vertices(std::vector<Vertex_def*>& vertices, Line_def* edge);

		/**
		 * runs over all vertices and tries to find all the possible rectangles.
		 * \param[out] possible_rectangles the list of rectangles found in the vertices list.
		 * \param[in] borders_param_lines the list of lines.
		 * \param[in] vertical a vector indicating the vertical, for oriented rectangle computation.
		 * \param[in] param_cos_ortho_tolerance the tolerance on the cosinus for orthogonal assertion.
		 *
		 */
		int find_all_possible_rectangles();

#ifdef _DEBUG_FUNCTIONS_
		/**
		 * select the rectangle that best describes the plane in the point cloud pc_plan.
		 *
		 * \param[in] possible_rectangles an array of rectangles to compare.
		 * \param[in] pc_plan the point cloud containing the plane we want to represent.
		 * \param[in] indices the indices of the points in pc_plan that actually represent the plane.
		 * \param[in] required_score the minimum score(in proportion of matching points) required
		 * to be considered as a describing rectangle.
		 * \param[in] lead_score the difference of score between 1st and 2nd best rectangles so that
		 * there is no possible ambiguity between who is the best.
		 * \param[in] n_samples the number of points to try.
		 * \param[out] best_score the best score.
		 * \param[out] best_index the index of the best rectangle in the \c possible_rectangles array.
		 * \param[out] not_matched_points the indices of the point that were not matched at least once.
		 * \return the index of the selected rectangle, or -1 if no rectangle satisfies the required_score and
		 * the lead_score constraints.
		 *
		 *
		 * Select randomly n_samples points in the input pc_plan
		 * and check for each rectangle if it contains this point
		 * If the best rectangle have a score > required_score
		 * and a <lead_score> points lead on the 2nd best, 
		 * we return it's index.
		 *
		 * \note the last 3 parameters are for debug. Use overriding function select_best_matching_rectangle() for general use.
		 *
		 **/
		int select_best_matching_rectangle(const std::vector<Rectangle>& possible_rectangles,pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr pc_plan, pcl::PointIndices::ConstPtr indices,float required_score,float lead_score, int n_samples, float& best_score,int& best_index,pcl::PointIndices::Ptr not_matched_points);

#endif


		/**
		 * select the rectangle that best describes the plane in the point cloud pc_plan.
		 *
		 * \param[in] possible_rectangles an array of rectangles to compare.
		 * \param[in] pc_plan the point cloud containing the plane we want to represent.
		 * \param[in] indices the indices of the points in pc_plan that actually represent the plane.
		 * \param[in] required_score the minimum score(in proportion of matching points) required
		 * to be considered as a describing rectangle.
		 * \param[in] lead_score the difference of score between 1st and 2nd best rectangles so that
		 * there is no possible ambiguity between who is the best.
		 * \param[in] n_samples the number of points to try.
		 * \return the index of the selected rectangle, or -1 if no rectangle satisfies the required_score and
		 * the lead_score constraints.
		 *
		 *
		 * Select randomly n_samples points in the input pc_plan
		 * and check for each rectangle if it contains this point
		 * If the best rectangle have a score > required_score
		 * and a <lead_score> points lead on the 2nd best, 
		 * we return it's index.
		 *
		 **/
		int select_best_matching_rectangle(const std::vector<Rectangle>& possible_rectangles,
				pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr pc_plan,
				pcl::PointIndices::ConstPtr indices,
				float required_score, float lead_score,
				int n_samples);

		/**
		 * returns a rectangle from the given vertices.
		 * \param[in] vertices_indices the the indices of the vertices of the rectangle to compute.
		 * \return a shared_ptr to the rectangle, as defined bellow.
		 *
		 * The returned rectangle is such that all points in it can
		 * be defined as \f$ a \cdot \overrightarrow{x} + b \cdot \overrightarrow{y}\f$
		 * with \f$(a,b) \geq (0,0)\f$ \n
		 * \f$\overrightarrow{x}\f$ (vect_x) and \f$\overrightarrow{y}\f$ (vect_y) are choosen so that \f$\overrightarrow{x} \times \overrightarrow{y}\f$ have (approx) the same direction 
		 * than the given vertical
		 **/
		boost::shared_ptr<Rectangle> compute_rectangle(const std::vector<int>& vertices_indices);

		/**
		 * Compute a transform so that the child frame have given x and y axis.
		 * \param[in] origin the origin of the child frame in the parent frame coordinates.
		 * \param[in] x_axis a vector indicating the desired x axis direction of the child frame.
		 * \param[in] y_axis idem.
		 * \return the transform, as described.
		 *
		 * \note axes CAN be not normalized
		 **/
		tf::Transform computeTransform(Eigen::Vector3f origin, Eigen::Vector3f x_axis,Eigen::Vector3f y_axis);

		/** compute the world's vertical reference from tf.
		 * \param[out] vertical a vector colinear to the z axis of the \c reference_tf_name expressed in the \c camera_tf_name coordinates.
		 * \param[out] origin the origin of the \c reference_tf_name frame in the \c camera_tf_name coordinates.
		 * \param[in] reference_tf_name the name of the reference (world) frame, in tf.
		 * \param[in] camera_tf_name the name of the camera frame in tf.
		 */
		static void get_vertical_referecence(tf::Vector3& vertical,tf::Vector3& origin, const std::string& reference_tf_name,const std::string& camera_tf_name);

};
#endif // _GEOMETRY_UTILITIES_H_

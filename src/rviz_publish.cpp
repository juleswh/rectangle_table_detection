#include "rviz_publish.h"

void publish_rviz_shape(ros::Publisher* marker_pub, float dim_x, float dim_y, float dim_z, bool remove_shape){
	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/plane_segmented";
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "basic_shapes";
	marker.id = 0;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::CUBE;

	// Set the marker action.  Options are ADD and DELETE
	if(remove_shape){
		marker.action = visualization_msgs::Marker::DELETE;
	}else{
		marker.action = visualization_msgs::Marker::ADD;

		// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
		marker.pose.position.x = dim_x/2;
		marker.pose.position.y = dim_y/2;
		marker.pose.position.z = dim_z/2;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 0.0;

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker.scale.x = dim_x;
		marker.scale.y = dim_y;
		marker.scale.z = dim_z;

		// Set the color -- be sure to set alpha to something non-zero!
		marker.color.r = 0.4f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = .3;

		marker.lifetime = ros::Duration();
		marker.frame_locked =true;
	}

	// Publish the marker
	marker_pub->publish(marker);
}

void publish_rviz_plane_norms(const ros::Publisher& marker_array_pub, const rectangular_table_detection::PlaneArray& planes_def, const std::string& frame_id){
	visualization_msgs::MarkerArray marker_array;
	visualization_msgs::Marker marker;

	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time::now();

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "visualization_marker_array";
	marker.id = 0;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::ARROW;

	// Set the marker action.  Options are ADD and DELETE
	marker.action = visualization_msgs::Marker::ADD;	

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.01;
	marker.scale.y = 0.01;
	marker.scale.z = 0.01;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.4f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = .3;

	marker.lifetime = ros::Duration(1.);

	//init the marker array to the number of planes and with the defaul marker defined above
	marker_array.markers.resize(planes_def.planes.size(),marker);

	std::vector<visualization_msgs::Marker>::iterator markers_it = marker_array.markers.begin();
	std::vector<rectangular_table_detection::Plane>::const_iterator planes_it = planes_def.planes.begin();

	while(markers_it!=marker_array.markers.end()){

		markers_it->id=markers_it-marker_array.markers.begin();
		markers_it->points.push_back(planes_it->point);
		geometry_msgs::Point second_point;
		second_point.x=planes_it->point.x+planes_it->normal.x/4;
		second_point.y=planes_it->point.y+planes_it->normal.y/4;
		second_point.z=planes_it->point.z+planes_it->normal.z/4;

		markers_it->points.push_back(second_point);

		++markers_it;
		++planes_it;
	}

	// Publish the marker
	marker_array_pub.publish(marker_array);
}

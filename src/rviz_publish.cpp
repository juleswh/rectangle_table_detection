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
	}

	// Publish the marker
	marker_pub->publish(marker);
}

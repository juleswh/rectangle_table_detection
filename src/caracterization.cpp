#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Point.h>

std::ofstream measures_csv;

void dimensions_cb(const geometry_msgs::Point& point){
	tf::TransformListener tf_listener;
	tf::StampedTransform transform;
	try{
		tf_listener.waitForTransform("/plane_segmented", "/world", ros::Time(0), ros::Duration(10.0) );
		tf_listener.lookupTransform("/plane_segmented" , "/world", ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return;
	}
	ROS_INFO("frame_ok");
	measures_csv << transform.getOrigin().x() << ','<< transform.getOrigin().y() << ','<< transform.getOrigin().z() << ',';
	measures_csv << transform.getRotation().x() << ',' << transform.getRotation().y() << ',' << transform.getRotation().z() << ',' << transform.getRotation().w() << ',';
	measures_csv << point.x <<','<<point.y<<';'<<std::endl;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "caracterization_table_detection");

	ros::NodeHandle node;

	measures_csv.open("measures.csv", std::ios::out | std::ios::trunc);
	if(measures_csv){
		measures_csv << "x,y,z,rx,ry,rz,rw,width,length;"<<std::endl;
	}else{
		ROS_ERROR("couldn't open file measures.csv in write mode");
		return 1;
	}

	ros::Subscriber sub = node.subscribe("/ransac_ortho_plane/table_dimensions", 1000, dimensions_cb);

	ros::spin();
	measures_csv.close();
	return 0;
}


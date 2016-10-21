#include <ros/ros.h>
#include <spencer_bridge/spencer_bridge.h>

int main(int argc, char** argv) {
	ros::init(argc,argv,"spencer_bridge");

	ROS_INFO("SPENCER_BRIDGE Init Spencer Bridge");
	ros::NodeHandle node_handle;

	ROS_INFO("SPENCER_BRIDGE Creating data reader");
	SpencerBridge spencer_bridge(node_handle);

	ROS_INFO("SPENCER_BRIDGE Created data reader. Starting loop");
	ros::Rate r(10);
	while (ros::ok()) {
		// ROS_INFO("SPENCER_BRIDGE Spinning");
		ros::spinOnce();
		// ROS_INFO("SPENCER_BRIDGE Spinned");
		spencer_bridge.publishData();
		// ROS_INFO("SPENCER_BRIDGE PUBLISHED data");
		r.sleep();
	}
}
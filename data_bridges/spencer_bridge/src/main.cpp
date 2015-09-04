#include <ros/ros.h>
#include <spencer_bridge/spencer_bridge.h>

int main(int argc, char** argv) {
	ros::init(argc,argv,"spencer_bridge");
	ros::NodeHandle node_handle;

	SpencerBridge spencer_bridge(node_handle);

	ros::Rate r(10);
	while (ros::ok()) {
		ros::spinOnce();
		spencer_bridge.publishData();
		r.sleep();
	}
}
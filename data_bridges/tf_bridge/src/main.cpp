#include <ros/ros.h>
#include <tf_bridge/tf_bridge.h>

int main (int argc,char** argv) {
	ros::init(argc,argv,"tf_bridge");
	ros::NodeHandle node_handle;

	TfBridge tf_bridge(node_handle);

	ros::Rate r(10);
	while (ros::ok()) {
		tf_bridge.getPoses();
		tf_bridge.publishData();
	}


	return 0;
}
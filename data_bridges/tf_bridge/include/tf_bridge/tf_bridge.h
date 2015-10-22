#ifndef TF_BRIDGE_H
#define TF_BRIDGE_H

#include <ros/ros.h>
#include <data_lib/data_lib.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <vector>
#include <map>

#include <situation_assessment_msgs/NamedPoseList.h>
#include <situation_assessment_msgs/NamedPose.h>
#include <situation_assessment_msgs/Group.h>
#include <situation_assessment_msgs/GroupList.h>

#include <geometry_msgs/Pose.h>

using namespace std;


class TfBridge:public DataLib {
public:
	TfBridge(ros::NodeHandle node_handle);
	void getPoses();
private:
	geometry_msgs::Pose tfToGeometry(tf::StampedTransform transform);

	tf::TransformListener listener_;

};

#endif

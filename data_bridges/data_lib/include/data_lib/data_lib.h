#ifndef DATALIB_H
#define DATALIB_H

#include <ros/ros.h>

#include <vector>
#include <map>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/foreach.hpp>

#include <situation_assessment_msgs/NamedPoseList.h>
#include <situation_assessment_msgs/NamedPose.h>
#include <situation_assessment_msgs/Group.h>
#include <situation_assessment_msgs/GroupList.h>



using namespace std;

// typedef map<string,geometry_msgs::Pose> PoseMap;



struct Entity {
	string name;
	string type;
	geometry_msgs::Pose pose;
};

typedef map<string,Entity> EntityMap;
typedef map<string,string> StringMap;
typedef map<string,vector<string> > StringVectorMap;

class DataLib {
public:
	DataLib(string bridge_name, ros::NodeHandle node_handle);
	void publishData();

protected:
	situation_assessment_msgs::NamedPoseList getNamedPoseListMsg(EntityMap entity_map);

	string bridge_name_;

	vector<string> agent_list_;
	vector<string> object_list_;
	string robot_name_;

	bool track_agents_;
	bool track_robot_;
	bool track_groups_;
	bool track_objects_;


	Entity robot_pose_;
	EntityMap agent_poses_;
	EntityMap group_poses_;
	EntityMap object_poses_;
	StringVectorMap agent_groups_;

	ros::Publisher robot_pub_,agents_pub_,objects_pub_,groups_pub_;


	ros::NodeHandle node_handle_;
};

#endif
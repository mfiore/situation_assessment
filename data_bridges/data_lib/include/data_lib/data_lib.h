/**
	data_lib.h
	author: Michelangelo Fiore

	defines the superclass used by other data bridges. The system is generic and can receive perception data from different
	sources, or bridges.
*/


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


//agents or objects
struct Entity {
	string name;
	string type;
	string category;
	geometry_msgs::Pose pose;
};

//useful typedefs
typedef map<string,Entity> EntityMap;
typedef map<string,string> StringMap;
typedef map<string,vector<string> > StringVectorMap;

class DataLib {
public:
	//the name is used to get parameters linked to the bridge
	DataLib(string bridge_name, ros::NodeHandle node_handle);
	void publishData();

protected:
	situation_assessment_msgs::NamedPoseList getNamedPoseListMsg(EntityMap entity_map); //poses of entities

	string bridge_name_;

	//list of agents and objects known by the system
	vector<string> agent_list_;
	vector<string> object_list_;
	string robot_name_;

	//each bridge can track or not agents, the robot, groups or objects
	bool track_agents_;
	bool track_robot_;
	bool track_groups_;
	bool track_objects_;

	//poses of the entities
	Entity robot_pose_;
	EntityMap agent_poses_;
	EntityMap group_poses_;
	EntityMap object_poses_;
	StringVectorMap agent_groups_;
	ros::Publisher robot_pub_,agents_pub_,objects_pub_,groups_pub_;


	ros::NodeHandle node_handle_;
};

#endif
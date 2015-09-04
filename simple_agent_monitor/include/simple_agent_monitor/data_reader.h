#ifndef DATA_READER_H
#define DATA_READER_H

#include <ros/ros.h>

#include <situation_assessment_msgs/NamedPose.h>
#include <situation_assessment_msgs/NamedPoseList.h>
#include <situation_assessment_msgs/Group.h>
#include <situation_assessment_msgs/GroupList.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point32.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <simple_agent_monitor/ring_buffer.h>

#include <boost/foreach.hpp>

#include <simple_agent_monitor/math_functions.h>

#include <simple_agent_monitor/entity.h>

typedef map<string,Entity> EntityMap;
typedef map<string,bool> BoolMap;
typedef map<string,vector<string> > StringVectorMap;


using namespace std;

class DataReader {
public:
	DataReader(ros::NodeHandle node_handle);
	EntityMap getAgentPoses();
	EntityMap getGroupPoses();
	StringVectorMap getAgentGroups();
	EntityMap getObjectPoses();
	Entity getRobotPoses();

private:
	void robotCallback(situation_assessment_msgs::NamedPose msg);
	void agentsCallback(situation_assessment_msgs::NamedPoseList msg);
	void objectsCallback(situation_assessment_msgs::NamedPoseList msg);
	void groupsCallback(situation_assessment_msgs::GroupList msg);

	void handleEntityMap(situation_assessment_msgs::NamedPoseList msg, EntityMap* map);


	ros::NodeHandle node_handle_;

	ros::Subscriber agents_sub_, groups_sub_, objects_sub_, robot_sub_;

	boost::mutex mutex_agent_poses_, mutex_group_poses_, mutex_object_poses_, mutex_robot_poses_;

	EntityMap agent_poses_map_, group_poses_map_, object_poses_map_;
	Entity robot_pose_;

	StringVectorMap agent_groups_map_;

	int ring_buffer_length_;

};

#endif
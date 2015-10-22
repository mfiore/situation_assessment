#include <data_lib/data_lib.h>

DataLib::DataLib(string bridge_name, ros::NodeHandle node_handle):bridge_name_(bridge_name),
node_handle_(node_handle) {

	string param_prefix="/situation_assessment/"+bridge_name_+"/";
	node_handle_.getParam(param_prefix+"track_robot",track_robot_);
	node_handle_.getParam(param_prefix+"track_agents",track_agents_);
	node_handle_.getParam(param_prefix+"track_objects",track_objects_);
	node_handle_.getParam(param_prefix+"track_groups",track_groups_);	
	node_handle_.getParam("/robot/name",robot_name_);
	node_handle_.getParam("/situation_assessment/human_names",agent_list_);
	node_handle_.getParam("/situation_assessment/object_names",object_list_);

	ROS_INFO("Parameters are:");
	ROS_INFO("Tracking robot %d:",track_robot_);
	ROS_INFO("Tracking agents %d:",track_agents_);
	ROS_INFO("Tracking objects %d:",track_objects_);
	ROS_INFO("Tracking groups %d:",track_groups_);
	ROS_INFO("Robot name %s",robot_name_.c_str());
	ROS_INFO("Known agents");
	BOOST_FOREACH(string agent, agent_list_) {
		ROS_INFO("- %s",agent.c_str());
	}	
	ROS_INFO("Known Objects");
	BOOST_FOREACH(string object, object_list_) {
		ROS_INFO("- %s",object.c_str());
	}


	if (track_robot_) {
		robot_pub_=node_handle_.advertise<situation_assessment_msgs::NamedPose>("situation_assessment/robot_pose",1000);
	}
	if (track_agents_) {
		agents_pub_=node_handle_.advertise<situation_assessment_msgs::NamedPoseList>("situation_assessment/agent_poses",1000);
	}
	if (track_objects_) {
		objects_pub_=node_handle_.advertise<situation_assessment_msgs::NamedPoseList>("situation_assessment/object_poses_",1000);
	}
	if (track_groups_) {
		groups_pub_=node_handle_.advertise<situation_assessment_msgs::GroupList>("situation_assessment/group_poses",1000);
	}
	ROS_INFO("Advertising appropriate topics");
}

void DataLib::publishData() {
	if (track_robot_) {

		situation_assessment_msgs::NamedPose msg;
		msg.name=robot_name_;
		msg.pose=robot_pose_.pose;
		msg.type="ROBOT";
		robot_pub_.publish(msg);
	}
	if (track_agents_){

		situation_assessment_msgs::NamedPoseList list_msg=getNamedPoseListMsg(agent_poses_);
		agents_pub_.publish(list_msg);
	}
	if (track_objects_) {
		situation_assessment_msgs::NamedPoseList list_msg=getNamedPoseListMsg(object_poses_);
		objects_pub_.publish(list_msg);
	}
	if (track_groups_) {
		situation_assessment_msgs::GroupList group_list_msg;
		situation_assessment_msgs::Group group_msg;

		vector<situation_assessment_msgs::Group> group_vector;
		for (EntityMap::iterator i=group_poses_.begin(); i!=group_poses_.end(); i++) {
			vector<string> members=agent_groups_[i->first];
			group_msg.members=members;
			group_msg.pose=i->second.pose;
 			group_msg.name=i->first;
 			group_vector.push_back(group_msg);
		}
		group_list_msg.list=group_vector;
		groups_pub_.publish(group_list_msg);
	}	
}


situation_assessment_msgs::NamedPoseList DataLib::getNamedPoseListMsg(EntityMap entity_map) {
	situation_assessment_msgs::NamedPoseList list_msg;
	vector<situation_assessment_msgs::NamedPose> msg_pose_vector;
	for (EntityMap::iterator i=entity_map.begin();i!=entity_map.end();i++) {
		situation_assessment_msgs::NamedPose entity_msg;
		entity_msg.name=i->first;
		entity_msg.pose=i->second.pose;
		entity_msg.type=i->second.type;
		msg_pose_vector.push_back(entity_msg);
	}
	list_msg.poses=msg_pose_vector;
	return list_msg;
}
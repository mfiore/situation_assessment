#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>
#include <vector>
#include <map>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "RingBuffer.h"
#include "situation_assessment_msgs/Fact.h"
#include "situation_assessment_msgs/FactList.h"
#include <situation_assessment_msgs/AgentPoseList.h>
#include "MathFunctions.h"
#include <utility>

using namespace std;

bool useHand=false;

string robotName;
vector<string> objects;
vector<string> agents;

double angleThreshold=0.5;

map<string,RingBuffer<geometry_msgs::Pose>> agentPose;
map<string,geometry_msgs::Pose> objectPose;
map<pair<string,string>,RingBuffer<double>> agentDistances;

map<string,vector<string>> groups;
ros::Publisher agentPosePublisher;

void setAgentPose(tf::StampedTransform transform, string name) {
	geometry_msgs::Pose pose;
	pose.position.x=transform.getOrigin().getX();
	pose.position.y=transform.getOrigin().getY();
	pose.position.z=transform.getOrigin().getZ();
	quaternionTFToMsg(transform.getRotation(),pose.orientation);
	if (agentPose[name].size==0) {
		agentPose[name].allocate(10);
	}

	agentPose[name].insert(pose);

}
void getPoses() {
	tf::TransformListener listener;
	//get human agents
	geometry_msgs::Pose groupPose;
	groupPose.position.x=0;
	groupPose.position.y=0;
	groupPose.position.z=0;

	for (string agentName:agents) {
		try {
			tf::StampedTransform transform;
			listener.waitForTransform("map", agentName, ros::Time(0), ros::Duration(1.0) );
			if (!ros::ok()) {
				return;
			}
			listener.lookupTransform("map",agentName,ros::Time(0),transform);

			setAgentPose(transform,agentName);
			groupPose.position.x=groupPose.position.x+transform.getOrigin().getX();
			groupPose.position.y=groupPose.position.y+transform.getOrigin().getY();
			groupPose.position.z=groupPose.position.z+transform.getOrigin().getZ();

		}     catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
	}
	groupPose.position.x=groupPose.position.x/nAgents;;
	groupPose.position.y=groupPose.position.y/nAgents;
	groupPose.position.z=groupPose.position.z/nAgents;;
	agentPose["GROUP_1"].insert(groupPose);
	for (int i=0; objects.size(); i++) {

		try {
			tf::StampedTransform transform;
			listener.waitForTransform("map", objects[i], ros::Time(0), ros::Duration(1.0) );
			if (!ros::ok()) {
				return;
			}
			listener.lookupTransform("map",objects[i],ros::Time(0),transform);

			geometry_msgs::Pose pose;
			pose.position.x=transform.getOrigin().getX();
			pose.position.y=transform.getOrigin().getY();
			pose.position.z=transform.getOrigin().getZ();
			quaternionTFToMsg(transform.getRotation(),pose.orientation);
			objectPose[objects[i]]=pose;

		}     catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
	}

	try {
		tf::StampedTransform transform;
		listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(1.0) );
		if (!ros::ok()) {
				return;
			}
		listener.lookupTransform("/map","base_link",ros::Time(0),transform);
		setAgentPose(transform,robotName);
	} catch(tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
	}
}

void publishPose() {
	situation_assessment_msgs::AgentPoseList list;
	for (auto agent: agentPose) {
		situation_assessment_msgs::AgentPose pose;
		pose.name=agent.first;
		pose.pose=agent.second.getSequence(1)[0];
		list.agentPoseList.push_back(pose);
	}
	agentPosePublisher.publish(list);

}

vector<situation_assessment_msgs::Fact> getDistances() {

	vector<situation_assessment_msgs::Fact> result;

	for (auto agent1:agentPose) {
		geometry_msgs::Pose pose1,pose2;
		pose1=agent1.second.getSequence(1)[0];
		for (auto agent2:agentPose) {

			if (agent2.first!=agent1.first) {
				pose2=agent2.second.getSequence(1)[0];
				double d=calculateDistance(pose1.position,pose2.position);
				pair<string,string> distanceInput{agent1.first,agent2.first};
				if (agentDistances[distanceInput].size==0) {
					agentDistances[distanceInput].allocate(10);
				}
				agentDistances[distanceInput].insert(d);
				situation_assessment_msgs::Fact f;
				f.subject=agent1.first;
				f.predicate.push_back("distance");
				f.predicate.push_back(agent2.first);
				f.value=to_string(d);
				result.push_back(f);
				cout<<"Distance "<<agent1.first<<" "<<agent2.first<<" "<<d<<"\n";
			}
		}
		for (string o:objects) {
			pose2=objectPose[o];
			double d=calculateDistance(pose1.position,pose2.position);
			pair<string,string> distanceInput{agent1.first,o};
			agentDistances[distanceInput].insert(d);

			situation_assessment_msgs::Fact f;
			f.subject=agent1.first;
			f.predicate.push_back("distance");
			f.predicate.push_back(o);
			f.value=to_string(d);
			result.push_back(f);

		}
	}
	return result;

}
vector<situation_assessment_msgs::Fact> getDeltaDistances() {
	vector<situation_assessment_msgs::Fact> result;
	for (auto agent1:agentPose) {
		geometry_msgs::Pose pose1,pose2;
		pose1=agent1.second.getSequence(1)[0];
		for (auto agent2:agentPose) {
			if (agent2.first!=agent1.first) {
				pose2=agent2.second.getSequence(1)[0];

				pair<string,string> distanceInput{agent1.first,agent2.first};
				vector<double> distances=agentDistances[distanceInput].getSequence(10);

				double deltaDistance=distances[9]-distances[0];
				situation_assessment_msgs::Fact f;
				f.subject=agent1.first;
				f.predicate.push_back("delta_distance");
				f.predicate.push_back(agent2.first);
				f.value=to_string(deltaDistance);
				result.push_back(f);
				cout<<"delta_distance "<<agent1.first<<" "<<agent2.first<<" "<<deltaDistance<<"\n";

			}
		}
		for (string o:objects) {
			pose2=objectPose[o];
			double d=calculateDistance(pose1.position,pose2.position);
			pair<string,string> distanceInput{agent1.first,o};
			vector<double> distances=agentDistances[distanceInput].getSequence(10);

			double deltaDistance=distances[9]-distances[0];

			situation_assessment_msgs::Fact f;
			f.subject=agent1.first;
			f.predicate.push_back("delta_distance");
			f.predicate.push_back(o);
			f.value=to_string(deltaDistance);
			result.push_back(f);

		}
	}
	return result;
}

vector<situation_assessment_msgs::Fact> getIsMoving() {
	vector<situation_assessment_msgs::Fact> result;
	for (auto agent1:agentPose) {
		vector<geometry_msgs::Pose> poses=agent1.second.getSequence(5);
		vector<geometry_msgs::Point> points;
		for (auto p:poses) {
			points.push_back(p.position);
		}
		bool value=isMoving(points);

		situation_assessment_msgs::Fact f;
		f.subject=agent1.first;
		f.predicate.push_back("isMoving");
		f.value=to_string(value);
		result.push_back(f);
		cout<<agent1.first<<" isMoving "<<value<<"\n";

	}
	return result;
}

vector<situation_assessment_msgs::Fact> getIsFacing() {
	vector<situation_assessment_msgs::Fact> result;
	for (auto agent1:agentPose) {
		geometry_msgs::Pose pose1,pose2;
		pose1=agent1.second.getSequence(1)[0];
		for (auto agent2:agentPose) {
			if (agent2.first!=agent1.first) {
				pose2=agent2.second.getSequence(1)[0];

				bool value=isFacing(pose1,pose2.position);
				situation_assessment_msgs::Fact f;
				f.subject=agent1.first;
				f.predicate.push_back("isFacing");
				f.predicate.push_back(agent2.first);
				f.value=to_string(value);
				result.push_back(f);
			}
		}

	}
	return result;
}


int main(int argc, char** argv) {


	nAgents=stoi(argv[1]);
	humanBaseName=argv[2];
	ros::init(argc,argv,"situation_assessment_msgs");


	n.getParam("/robot_name",robotName);
	n.getParam("/human_agents/agentNames",agents);


	ros::NodeHandle n;
	agentPosePublisher=n.advertise<situation_assessment_msgs::AgentPoseList>("situation_assessment/agent_pose_list",1000);
	ros::Publisher factPublisher=n.advertise<situation_assessment_msgs::FactList>("situation_assessment/agent_fact_list",1000);

	for (string agentName:agents) {
	groups["GROUP_1"].push_back(agentName);
	}
	agentPose["GROUP_1"].allocate(10);

	ros::Rate rate(10.0);
	while (ros::ok()) {

		getPoses();
		publishPose();
		vector<situation_assessment_msgs::Fact> distances=getDistances();
		vector<situation_assessment_msgs::Fact> isMoving=getIsMoving();
		vector<situation_assessment_msgs::Fact> delta_distance=getDeltaDistances();


		situation_assessment_msgs::FactList factList;
		factList.factList=isMoving;
		factList.factList.insert(factList.factList.end(),distances.begin(),distances.end());
		factList.factList.insert(factList.factList.end(),delta_distance.begin(),delta_distance.end());

		factPublisher.publish(factList);

		rate.sleep();
	}
	ros::shutdown();

}

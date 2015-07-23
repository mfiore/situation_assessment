#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include "boost/thread.hpp"
#include <utility>
#include <tuple>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <situation_assessment_msgs/MonitorRequest.h>
#include <situation_assessment_msgs/EmptyRequest.h>


using namespace std;
double threshold=0.2;

ros::Publisher monitorPub;

map<string,geometry_msgs::PoseStamped> lastPose;

vector<string> agentNames;
vector<string> agentHands;

map<string,geometry_msgs::Point> monitors;


geometry_msgs::Point getPosition(string object, string *returnReport) {
	geometry_msgs::Point position;

	return position;
}

bool addMonitor(situation_assessment_msgs::MonitorRequest::Request  &req,
             situation_assessment_msgs::MonitorRequest::Response &res) {
	string report;
	string object=req.object
	geometry_msgs::Point position=getPosition(object, &report);
	if (report=="OK") {
		ROS_INFO("Adding monitor for object %s",object.c_str());
		monitors[object]=position;
		res.report=true;
	}
	else {
		ROS_INFO("Couldn't add monitor for object %s",object.c_str());
		res.report=false;
	}
	return true;

}

bool removeMonitor(situation_assessment_msgs::MonitorRequest::Request  &req,
             situation_assessment_msgs::MonitorRequest::Response &res) {
	string object=req.object

	monitors.erase(object);
	ROS_INFO("Removing monitor for object %s",object.c_str());
	res.report=true;
	return true;

}

bool removeAllMonitors(situation_assessment_msgs::EmptyRequest::Request  &req,
             situation_assessment_msgs::EmptyRequest::Response &res){
	monitors.clear();
	ROS_INFO("Removing all monitors %s",object.c_str());
	res.report=true;
}

void getAgentPose() {
		

	for (string hand:agentHands) {
		tf::StampedTransform transform;
		ros::Time now = ros::Time(0);
		tf::TransformListener listener;


		try {//
//			//transform from the mocap frame to map
			listener.waitForTransform("/map", hand,
					now, ros::Duration(3.0));
			listener.lookupTransform("/map", hand,
					now, transform);
				lastPose[hand].pose.position.x=transform.getOrigin().getX();
				lastPose[hand].pose.position.y=transform.getOrigin().getY();
				lastPose[hand].pose.position.z=transform.getOrigin().getZ();

		} catch (tf::TransformException ex) {
			ROS_ERROR("%s", ex.what());
	}

}


double dist3d(string object, string hand) {
	geometry_msgs::Point objectPoint=monitors[object];
	geometry_msgs::Point agentPoint=lastPose[hand].pose.position;
	//cout<<agentPoint.x<<" "<<agentPoint.y<<" "<<objectPoint.x<<" "<<objectPoint.y<<"\n";
	double dist=sqrt(pow(objectPoint.x - agentPoint.x,2)+pow(objectPoint.y-agentPoint.y,2)+pow(objectPoint.z-agentPoint.z,2));
	return dist;
}



int main(int argc, char **argv) {
	ros::init(argc,argv,"human_monitors");
	ros::NodeHandle n;

	
	n.getParam("/human_agents/agentNames",agents);
	for (int i=0; i<agents.size(); i++) {
		string handParam="/human_agents/"+agents[i]+"/right_hand";
		string hand;
		n.getParam(handParam,hand);
		agentHands.push_back(hand);
	}


	monitorPub=n.advertise<situation_assessment_msgs:MonitorResult>("situation_assessment/monitor_status");
   
    ros::ServiceServer addMonitorServer = n.advertiseService("situation_assessment/addMonitor", addMonitor);
    ros::ServiceServer addMonitorServer = n.advertiseService("situation_assessment/removeMonitor", removeMonitor);
    ros::ServiceServer addMonitorServer = n.advertiseService("situation_assessment/removeAllMonitors", removeAllMonitors);
	ros::Rate r(5);
	while(ros::ok()) {
		getAgentPose();
		for (string hand: agentHands) {
			double distMin=10;
			string minMonitor;
			bool isActive=false;
			for (auto monitor:monitors) {
				double dist=dist3d(monitor.first,hand);
					if (dist<threshold && dist<distMin) {
					isActive=true;
					minMonitor=monitor.first;
					distMin=dist;
				}
				ROS_INFO("%s distance to %s %d",monitor.first.c_str(),hand.c_str(),dist);
			}
			if (isActive) {
				trigger(minMonitor);
			}
		}
		r.sleep();
	}
}




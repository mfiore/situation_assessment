#include "ros/ros.h"

#include <iostream>
#include "situation_assessment_msgs/FactList.h"
#include "situation_assessment_msgs/DatabaseRequest.h"

using namespace std;

int main(int argc, char ** argv) {
	ros::init(argc,argv,"init_database");

	ROS_INFO("Started module init_database");
	ros::NodeHandle node_handle;

	ros::ServiceClient add_client=node_handle.serviceClient<situation_assessment_msgs::DatabaseRequest>("situation_assessment/add_facts");

	situation_assessment_msgs::DatabaseRequest database_request_srv;

	ROS_INFO("Registered to service add_facts");

	situation_assessment_msgs::FactList fact_list;
	situation_assessment_msgs::Fact f;
	f.model="PR2_ROBOT";
	f.subject="PR2_ROBOT";
	f.predicate.push_back("isAt");
	f.value="TABLE_4";

	fact_list.fact_list.push_back(f);

	database_request_srv.request.fact_list=fact_list.fact_list;
	add_client.call(database_request_srv);

	ROS_INFO("Added a fact to the database");

	return 0;


}
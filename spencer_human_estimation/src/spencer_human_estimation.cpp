#include <ros/ros.h>

//services
#include <situation_assessment_msgs/QueryDatabase.h>

//messages
#include <situation_assessment_msgs/FactList.h>
#include <situation_assessment_msgs/Fact.h>
#include <situation_assessment_msgs/HumanIntention.h>
#include <situation_assessment_msgs/HumanIntentionList.h>


#include <boost/foreach.hpp>

#include <math.h>

using namespace std;

string robot_name;
bool use_orientation;

vector<string> getInformationScreens(ros::ServiceClient* simple_database_client) {
	vector<string> information_screens;

	situation_assessment_msgs::Fact request_fact;
	situation_assessment_msgs::QueryDatabase query;

	request_fact.predicate.push_back("type");
	request_fact.value="information_screen";
	query.request.query=request_fact;
	if (simple_database_client->call(query)) {
		vector<situation_assessment_msgs::Fact> result=query.response.result;
		BOOST_FOREACH(situation_assessment_msgs::Fact f,result) {
			information_screens.push_back(f.subject);
		}
	}
	else {
		ROS_WARN("Failed to contact database");
	}
	return information_screens;

}

vector<situation_assessment_msgs::HumanIntention> getWantInformation(
	ros::ServiceClient* simple_database, vector<string> information_screens) {
	vector<situation_assessment_msgs::HumanIntention> result;

	situation_assessment_msgs::QueryDatabase request_area,request_orientation;
	request_area.request.query.model=robot_name;
	request_area.request.query.predicate.push_back("isInArea");

	request_orientation.request.query.model=robot_name;
	request_orientation.request.query.predicate.push_back("isFacing");

	ROS_INFO("First Loop size is %d",information_screens.size());

	BOOST_FOREACH(string screen, information_screens) {
		//first we get all the people in an information screen area
		request_area.request.query.value=screen;
		if(simple_database->call(request_area)){
			vector<situation_assessment_msgs::Fact> humans_in_area=request_area.response.result;
			ROS_INFO("Loop size is %d",humans_in_area.size());
			BOOST_FOREACH(situation_assessment_msgs::Fact human,humans_in_area) {
				if (use_orientation) {
					//now we get the people that are looking at the screen
					request_orientation.request.query.subject=human.subject;
					request_orientation.request.query.value=screen;
					simple_database->call(request_orientation);
					if (request_orientation.response.result.size()>0) {
						situation_assessment_msgs::HumanIntention intention;
						intention.name=human.subject;
						intention.intention="wants_informations";

						result.push_back(intention);
					}	
				}
				else {
					situation_assessment_msgs::HumanIntention intention;
					intention.name=human.subject;
					intention.intention="wants_informations";	
					result.push_back(intention);		
				}
			}
		}	
		else {
				ROS_ERROR("Failed to contact database");
				break;
		}
	}
	ROS_INFO("Returning");
	return result;
}

int main(int argc,char** argv) {
	ros::init(argc,argv,"simple_human_estimation");
	ros::NodeHandle node_handle;

	ROS_INFO("Init simple_human_estimation");

	node_handle.getParam("/robot/name",robot_name);
	node_handle.getParam("/situation_assessment/human_estimation/use_orientation",use_orientation);

	ros::ServiceClient database_client=node_handle.serviceClient<situation_assessment_msgs::QueryDatabase>("situation_assessment/query_database",true);
	ROS_INFO("Waiting for query database service");
	database_client.waitForExistence();

	ros::Rate r(3);

	ros::Publisher pub=node_handle.advertise<situation_assessment_msgs::HumanIntentionList>("situation_assessment/human_intentions",true);
	ROS_INFO("Started publishing intention list");

	while (ros::ok()) {
		situation_assessment_msgs::HumanIntentionList intention_list;;

		vector<string> information_screens=getInformationScreens(&database_client);

		vector<situation_assessment_msgs::HumanIntention> want_informations=getWantInformation(&database_client,information_screens);		
		intention_list.intention_list=want_informations;

		pub.publish(intention_list);
		r.sleep();
	}


}
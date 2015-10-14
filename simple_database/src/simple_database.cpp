/*
 * DatabaseInterface.cpp
 *
 *  Created on: Jun 3, 2015
 *      Author: mfiore
 */


#include "ros/ros.h"
#include <iostream>
#include <map>
#include <utility>
#include <string>

#include "boost/thread.hpp"

#include "simple_database/database.h"
#include "simple_database/database_element.h"

#include "situation_assessment_msgs/QueryDatabase.h"
#include "situation_assessment_msgs/Fact.h"
#include "situation_assessment_msgs/FactList.h"
#include "situation_assessment_msgs/DatabaseRequest.h"

using namespace std;

Database database;

ros::Publisher world_status_publisher;

string robot_name;

bool query(situation_assessment_msgs::QueryDatabase::Request &req,
        situation_assessment_msgs::QueryDatabase::Response &res) {

	string predicate_string="";
	for (int i=0; i<req.query.predicate.size();i++) {
		predicate_string=predicate_string+req.query.predicate[i]+" ";
	}
	ROS_INFO("Received query %s %s %s %s ",req.query.model.c_str(), req.query.subject.c_str(),
			predicate_string.c_str(),  req.query.value[0].c_str() );
	DatabaseElement element(req.query.model, req.query.subject, req.query.predicate, req.query.value);
	vector<DatabaseElement> result = database.getElements(element);
	vector<situation_assessment_msgs::Fact> return_facts;
	ROS_INFO("Return");
	for (DatabaseElement el : result) {
        situation_assessment_msgs::Fact f;
        f.model = el.model_;
        f.subject = el.subject_;
        f.predicate = el.predicate_;
        f.value = el.value_;
        return_facts.push_back(f);

        // ROS_INFO("%s %s %s ",f.model.c_str(), f.subject.c_str()
        		// , f.value.c_str() );
    }
    res.result = return_facts;

    return true;
}


// void simpleAgentMonitorCallback(const situation_assessment_msgs::FactList::ConstPtr& msg) {
//     vector<situation_assessment_msgs::Fact> v_fact=msg->fact_list;
// 	for (auto f:v_fact) {
// 		DatabaseElement element(robot_name,f.subject,f.predicate,f.value);
//         vector<DatabaseElement> found_elements=database.getElements(element);

//         if (found_elements.size()==0) {
//             if (f.predicate[0]=="isMoving") {
//                 cout<<"an isMoving noDuplicate\n";
//             }
//             DatabaseElement element2(robot_name,f.subject,f.predicate,"");
//             vector<DatabaseElement> found_elements2=database.getElements(element);
//             if (found_elements2.size()!=0) {
//                 cout<<"found element with "<<found_elements2[0].value_<<"\n";
//                 database.removeElement(found_elements2[0]);
//             }

//         database.addElement(element);

//         }
//         else {
//             cout<<found_elements[0].value_<<"\n";
//             cout<<"Duplicate\n";
//         }
//     }

// }

bool addFacts(situation_assessment_msgs::DatabaseRequest::Request &req,
        situation_assessment_msgs::DatabaseRequest::Response &res) {
    ROS_INFO("Received request to add facts:");
    for (situation_assessment_msgs::Fact fact:req.fact_list) {
        ROS_INFO("%s %s %s %s",fact.model.c_str(),fact.subject.c_str(),fact.predicate[0].c_str(),fact.value[0].c_str());
        DatabaseElement element(fact.model.c_str(),fact.subject,fact.predicate,fact.value);

        vector<DatabaseElement> found_elements=database.getElements(element);
        if (found_elements.size()==0) {
         database.addElement(element);
        }
    }
    return true;
}

bool removeFacts(situation_assessment_msgs::DatabaseRequest::Request &req,
        situation_assessment_msgs::DatabaseRequest::Response &res) {
    ROS_INFO("Received request to remove facts");
    // ROS_INFO("Database size before remove is %ld",database.database_.size());
    for (situation_assessment_msgs::Fact fact:req.fact_list) {
        ROS_INFO("Remove %s %s %s %s",robot_name.c_str(),fact.subject.c_str(),fact.predicate[0].c_str(),fact.value[0].c_str());
        DatabaseElement element(robot_name,fact.subject,fact.predicate,fact.value);
        database.removeElement(element);
    }
    // ROS_INFO("Database size after remove is %ld",database.database_.size());

    return true;

}


void publishWorldStatus() {
    situation_assessment_msgs::FactList fact_list;
    DatabaseElement empty_element("","",vector<string>(),vector<string>());

    ros::Rate r(1);
    while (ros::ok()) {


        vector<DatabaseElement> elements=database.getElements(empty_element);

        for (DatabaseElement db_element:elements) {
            situation_assessment_msgs::Fact fact;
            fact.model=db_element.model_;
            fact.subject=db_element.subject_;
            fact.predicate=db_element.predicate_;
            fact.value=db_element.value_;

            fact_list.fact_list.push_back(fact);
        }

        world_status_publisher.publish(fact_list);

        r.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "simple_database");
    ros::NodeHandle node_handle;

    node_handle.getParam("/robot/name",robot_name);

    ROS_INFO("Init simple_database");

    ros::ServiceServer service_add = node_handle.advertiseService("situation_assessment/add_facts", addFacts);
    ros::ServiceServer service_remove = node_handle.advertiseService("situation_assessment/remove_facts", removeFacts);
    ros::ServiceServer service_query = node_handle.advertiseService("situation_assessment/query_database", query);

    ROS_INFO("Advertising services");
    world_status_publisher=node_handle.advertise<situation_assessment_msgs::FactList>("situation_assessment/world_status",1000);

    // Server action_server(n, "situation_assessment/monitor_facts", 
    // boost::bind(&monitorFacts,&action_server), false);
    // ROS_INFO("Ready to monitor facts");

    // ros::Subscriber sub = node_handle.subscribe("/situation_assessment/agentFactList", 1000, simpleAgentMonitorCallback);


    ROS_INFO("Started database");

    boost::thread t(&publishWorldStatus);

    ros::spin();
  	ros::waitForShutdown();
    return 0;
}


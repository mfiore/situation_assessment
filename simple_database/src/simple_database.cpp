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
    ROS_INFO("SIMPLE_DATABASE Got query");

	string predicate_string="";
	string value_string="";
    for (int i=0; i<req.query.predicate.size();i++) {
		predicate_string=predicate_string+req.query.predicate[i]+" ";
	}
    for (int i=0;i<req.query.value.size();i++) {
        value_string=value_string+req.query.value[i]+" ";
    }
	ROS_INFO("SIMPLE_DATABASE Received query %s %s %s %s ",req.query.model.c_str(), req.query.subject.c_str(),
			predicate_string.c_str(),  value_string.c_str() );
	DatabaseElement element(req.query.model, req.query.subject, req.query.predicate, req.query.value);
	vector<DatabaseElement> result = database.getElements(element);
	vector<situation_assessment_msgs::Fact> return_facts;
	ROS_INFO("SIMPLE_DATABASE Return %ld elements",result.size());
	for (int i=0; i<result.size();i++) {
        DatabaseElement el=result[i];
        situation_assessment_msgs::Fact f;
        f.model = el.model_;
        f.subject = el.subject_;
        f.predicate = el.predicate_;
        f.value = el.value_;
        return_facts.push_back(f);

        string res_predicate_string, res_value_string;
        res_predicate_string="";
        res_value_string="";
        for (int i=0; i<f.predicate.size();i++) {
            res_predicate_string=res_predicate_string+f.predicate[i];
        }        
        for (int i=0; i<f.value.size();i++) {
            res_value_string=res_value_string+f.value[i];
        }

        ROS_INFO("SIMPLE_DATABASE %s %s %s ",f.model.c_str(), f.subject.c_str()
        		,res_predicate_string.c_str(), res_value_string.c_str() );
    }
    res.result = return_facts;

    return true;
}




bool addFacts(situation_assessment_msgs::DatabaseRequest::Request &req,
        situation_assessment_msgs::DatabaseRequest::Response &res) {
    ROS_INFO("SIMPLE_DATABASE Received request to add facts:");
    for (int i=0; i<req.fact_list.size();i++) {
        situation_assessment_msgs::Fact fact=req.fact_list[i];
        // ROS_INFO("SIMPLE_DATABASE %s %s %s %s",fact.model.c_str(),fact.subject.c_str(),fact.predicate[0].c_str());
        // ROS_INFO("SIMPLE_DATABASE %s",fact.value[0].c_str());
        DatabaseElement element(fact.model,fact.subject,fact.predicate,fact.value);

        vector<DatabaseElement> found_elements=database.getElements(element);
        if (found_elements.size()==0) {
         database.addElement(element);
        }
    }
    ROS_INFO("SIMPLE_DATABASE Finishing with adding facts");
    return true;
}

bool removeFacts(situation_assessment_msgs::DatabaseRequest::Request &req,
        situation_assessment_msgs::DatabaseRequest::Response &res) {
    ROS_INFO("SIMPLE_DATABASE Received request to remove facts");
    // ROS_INFO("SIMPLE_DATABASE Database size before remove is %ld",database.database_.size());
    for (int i=0; i<req.fact_list.size();i++) {
        situation_assessment_msgs::Fact fact=req.fact_list[i];
        // ROS_INFO("SIMPLE_DATABASE Remove %s %s %s %s",robot_name.c_str(),fact.subject.c_str(),fact.predicate[0].c_str(),fact.value[0].c_str());
        DatabaseElement element(fact.model,fact.subject,fact.predicate,fact.value);
        database.removeElement(element);
    }
    // ROS_INFO("SIMPLE_DATABASE Database size after remove is %ld",database.database_.size());

    return true;

}


void publishWorldStatus() {
    situation_assessment_msgs::FactList fact_list;
    DatabaseElement empty_element("","",vector<string>(),vector<string>());

    ros::Rate r(1);
    while (ros::ok()) {


        vector<DatabaseElement> elements=database.getElements(empty_element);

        for (int i=0; i<elements.size();i++) {
            DatabaseElement db_element=elements[i];
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

    ROS_INFO("SIMPLE_DATABASE Init simple_database");

    ros::ServiceServer service_add = node_handle.advertiseService("situation_assessment/add_facts", addFacts);
    ros::ServiceServer service_remove = node_handle.advertiseService("situation_assessment/remove_facts", removeFacts);
    ros::ServiceServer service_query = node_handle.advertiseService("situation_assessment/query_database", query);

    ROS_INFO("SIMPLE_DATABASE Advertising services");
    world_status_publisher=node_handle.advertise<situation_assessment_msgs::FactList>("situation_assessment/world_status",1000);



    ROS_INFO("SIMPLE_DATABASE Started database");

    boost::thread t(&publishWorldStatus);

    ros::spin();
  	ros::waitForShutdown();
    return 0;
}


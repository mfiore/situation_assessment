/* 
 * File:   DatabaseTester.cpp
 * Author: mfiore
 * 
 * Created on February 4, 2015, 5:39 PM
 */

#include <ros/service_client.h>
#include "ros/ros.h"

#include "situation_assessment_msgs/QueryDatabase.h"
#include "situation_assessment_msgs/Fact.h"
#include "std_msgs/String.h"
#include <vector>
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "database_tester");

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<situation_assessment_msgs::QueryDatabase>("situation_assessment/query_database");
    situation_assessment_msgs::QueryDatabase srv;
    situation_assessment_msgs::Fact fact;

    fact.model = "";
    fact.subject = "";

    // fact.value = "";

    srv.request.query = fact;
    ROS_INFO("sending request\n");
    client.call(srv);
    vector<situation_assessment_msgs::Fact> result = srv.response.result;
    ROS_INFO("received response\n");
    for (situation_assessment_msgs::Fact f : result) {
     
    //    cout << f.model << " " << f.subject << " " << f.predicate << " " << f.value << "\n";
    }


}

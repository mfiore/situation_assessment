/**
    agent_monitors.h
    author: Michelangelo Fiore

    encapsulate procedures to calculate geometrical information from perception data of agents.

*/

#ifndef AGENT_MONITORS_H
#define AGENT_MONITORS_H

#include <ros/ros.h>
#include <map>
#include <vector>
#include <string>
#include <boost/polygon/polygon.hpp>
#include <boost/foreach.hpp>

#include <simple_agent_monitor/entity.h>

#include <simple_agent_monitor/math_functions.h>

#include <situation_assessment_msgs/Fact.h> 

namespace gtl = boost::polygon;

typedef gtl::polygon_data<double> Polygon;
typedef gtl::polygon_traits<Polygon>::point_type Point;


using namespace std;

typedef map<string,Entity> EntityMap;
typedef map<string,bool> BoolMap;
typedef map<string,vector<string> > StringVectorMap;
typedef map<string,Polygon> PolygonMap; 
typedef map<pair<string, string> ,RingBuffer<double> >  PairMap; 


class AgentMonitors {
public:
	AgentMonitors(string robot_name);

    vector<situation_assessment_msgs::Fact> calculateIsFacing(EntityMap map1, EntityMap map2); //which entity an agent is facing
    vector<situation_assessment_msgs::Fact> getGroupContains(StringVectorMap map); //members of groups
    vector<situation_assessment_msgs::Fact> getDistances(EntityMap map1, EntityMap map2, PairMap* 
    	entity_distances); //Distances between entities in number
    vector<situation_assessment_msgs::Fact> getIsMoving(EntityMap map); //calculates if entities are moving or not
    
    vector<situation_assessment_msgs::Fact> getEntityType(EntityMap map); //objects, agents or what

    vector<situation_assessment_msgs::Fact> getDeltaDistances(EntityMap map1, EntityMap map2,
    PairMap entity_distances); //delta distances of agents in numbers
    vector<situation_assessment_msgs::Fact> getIsInArea(EntityMap map, PolygonMap areas);  //which agents are present in areas

    // vector<situation_assessment_msgs::Fact> getObjectTypes(EntityMap map); //types of objects


	private:
        string robot_name_;
};
#endif
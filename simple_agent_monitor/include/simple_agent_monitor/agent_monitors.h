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

    vector<situation_assessment_msgs::Fact> calculateIsFacing(EntityMap map1, EntityMap map2);
    vector<situation_assessment_msgs::Fact> getGroupContains(StringVectorMap map);
    vector<situation_assessment_msgs::Fact> getDistances(EntityMap map1, EntityMap map2, PairMap* 
    	entity_distances);
    vector<situation_assessment_msgs::Fact> getIsMoving(EntityMap map);
    
    vector<situation_assessment_msgs::Fact> getEntityType(EntityMap map);

    vector<situation_assessment_msgs::Fact> getDeltaDistances(EntityMap map1, EntityMap map2,
    PairMap entity_distances);
    vector<situation_assessment_msgs::Fact> getIsInArea(EntityMap map, PolygonMap areas); 


    vector<situation_assessment_msgs::Fact> getObjectTypes(EntityMap map);


	private:
        string robot_name_;
};
#endif
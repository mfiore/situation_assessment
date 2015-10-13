/**
    simple_agent_onitor.h
    author: Michelangelo Fiore

	class to apply geometrical reasoning on entities in the system, creating facts containing the distance between agents,
	delta distances, facing and so on.
*/

//ros stuff
#include <ros/ros.h>

//msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>

#include "situation_assessment_msgs/Fact.h"
#include "situation_assessment_msgs/FactList.h"
#include "situation_assessment_msgs/AreaList.h"

//services
#include <situation_assessment_msgs/AddArea.h>
#include <situation_assessment_msgs/NameRequest.h>
#include <situation_assessment_msgs/DatabaseRequest.h>

// //boost
#include <boost/polygon/polygon.hpp>

//other
#include <utility>
#include <string>
#include <vector>
#include <map>

#include "simple_agent_monitor/data_reader.h"
#include "simple_agent_monitor/agent_monitors.h"

using namespace std;
namespace gtl = boost::polygon;

//useful typedef
typedef map<string,RingBuffer<geometry_msgs::Pose>> BufferMap;
typedef map<string,bool> BoolMap;
typedef map<string,vector<string> > StringVectorMap;
typedef map<string,Polygon> PolygonMap; 
typedef map<pair<string, string> ,RingBuffer<double> > PairMap; 
typedef map<string,geometry_msgs::Polygon> GeometryPolygonMap;

//parameters
string robotName;
double angleThreshold;

PairMap agentDistances;  //distances between agents
PolygonMap areas; 		  //areas represented as polygons
GeometryPolygonMap msg_areas_map;   //areas as geometry msgs, used for publishing
GeometryPolygonMap entity_areas;	//areas linked to entities


//service to add a geometrical area in the environment
bool addArea(situation_assessment_msgs::AddArea::Request &req, situation_assessment_msgs::AddArea::Response &res) {
	using namespace boost::polygon::operators;

	if (req.linked_to_entity!="") { //if linked will update the area with the entity position
		entity_areas[req.linked_to_entity]=req.area;
		ROS_INFO("Adding a new area to entity %s",req.linked_to_entity.c_str());
	}
	else {
		geometry_msgs::Polygon area=req.area;

		ROS_INFO("Adding a new area to monitor. Coordinates:");	
		Point pts[area.points.size()];
		int i=0;
		for (geometry_msgs::Point32 point:area.points) {
			pts[i]=gtl::construct<Point>(point.x,point.y);
			ROS_INFO("- %f %f",point.x,point.y);
			i++;
		}
		Polygon poly;
		gtl::set_points(poly, pts, pts+i);

		areas[req.name]=poly;
		msg_areas_map[req.name]=req.area;
	}

	res.result=true;
	return true;
}

//removes an area
bool removeArea(situation_assessment_msgs::NameRequest::Request &req, situation_assessment_msgs::NameRequest::Response &res) {
	areas.erase(req.name);

	msg_areas_map.erase(req.name);
	res.result=true;
	return true;
}

//compare facts to check if they are the same or different
bool compareFacts(situation_assessment_msgs::Fact f1, situation_assessment_msgs::Fact f2) {
	if (f1.predicate.size()!=f2.predicate.size()) return false;
	for (int i=0; i<f1.predicate.size();i++) {
		if (f1.predicate[i]!=f2.predicate[i]) return false; 
	}
	return f1.subject==f2.subject && f1.model==f2.model && f1.value==f2.value;
}
//finds a fact in the list
int findFact(situation_assessment_msgs::Fact f, vector<situation_assessment_msgs::Fact> list) {
	for (int i=0; i<list.size();i++) {
		if (compareFacts(list[i],f)) return i;
	}
	return -1;
}

//updates the database with new facts, removing obsolete facts
void updateDatabase(ros::ServiceClient* add_database_client,ros::ServiceClient* remove_database_client,
	vector<situation_assessment_msgs::Fact> factList, vector<situation_assessment_msgs::Fact> old_fact_list) {
	map<int,bool> old_fact_found;
	vector<situation_assessment_msgs::Fact> to_add,to_remove;

	//start setting old facts as obsolete. When we found them (e.g. they are not obsolete) we update the value to true in this map
	for (int i=0; i<old_fact_list.size();i++) {
		old_fact_found[i]=false;
	}

	//if we find a fact in the new fact list which wasn't in the old, we add it to the list of facts to add to the database
	for (int i=0; i<factList.size();i++) {
		int pos=findFact(factList[i],old_fact_list);
		if (pos==-1) {
			to_add.push_back(factList[i]);
		}
		else {
			old_fact_found[pos]=true;
		}
	}
	//every fact of the old list that's not in the new will be added to the list of facts to remove from the atabase.
	for (int i=0;i<old_fact_list.size();i++) {
		if (old_fact_found[i]==false) {
			to_remove.push_back(old_fact_list[i]);
		}
	}
	situation_assessment_msgs::DatabaseRequest req_add,req_remove;
	req_add.request.fact_list=to_add;
	req_remove.request.fact_list=to_remove;
	if (!add_database_client->call(req_add)) {
		ROS_WARN("Can't add facts to database");
	}
	if (!remove_database_client->call(req_remove)) {
		ROS_WARN("Cant remove facts from database");
	} 
}

//rotates a point by angle theta around the pivot point
geometry_msgs::Point32 rotatePoint(geometry_msgs::Point32 p, geometry_msgs::Point32 pivot, double theta) {

	p.x=p.x-pivot.x;
	p.y=p.y-pivot.y;

	geometry_msgs::Point32 rotated_p;

	rotated_p.x=p.x*cos(theta)-p.y*sin(theta);
	rotated_p.y=p.x*sin(theta)+p.y*cos(theta);

	rotated_p.x=rotated_p.x+pivot.x;
	rotated_p.y=rotated_p.y+pivot.y;

	return rotated_p;
}


//updates areas linked to entities with their new positions
void updateEntityAreas(EntityMap agent_poses) {
	for(GeometryPolygonMap::iterator it=entity_areas.begin();it!=entity_areas.end();it++) {
		string entity_name=it->first;
		geometry_msgs::Polygon area=it->second;


		if (agent_poses.find(entity_name)!=agent_poses.end()) {
			Entity this_entity=agent_poses[entity_name];
			
			geometry_msgs::Pose this_entity_pose=this_entity.pose.getSequence(1)[0];
			geometry_msgs::Point32 entity_center;
			entity_center.x=this_entity_pose.position.x; 
			entity_center.y=this_entity_pose.position.y;
			double entity_orientation=tf::getYaw(this_entity_pose.orientation)-1.6; 
			for (int i=0; i<area.points.size();i++) {
				area.points[i].x+=this_entity_pose.position.x;
				area.points[i].y+=this_entity_pose.position.y;

				area.points[i]=rotatePoint(area.points[i],entity_center,entity_orientation);
			}


			Point pts[area.points.size()];
			int i=0;
			for (geometry_msgs::Point32 point:area.points) {
				pts[i]=gtl::construct<Point>(point.x,point.y);
				i++;
			}
			Polygon poly;
			gtl::set_points(poly, pts, pts+i);

			areas[entity_name]=poly;
			msg_areas_map[entity_name]=area;
		}
		else {
			if (areas.find(entity_name)!=areas.end()) {
				areas.erase(entity_name);
				msg_areas_map.erase(entity_name);
			}
		}
	}

}

int main(int argc, char** argv) {

	ros::init(argc,argv,"simple_agent_monitor");

	ros::NodeHandle node_handle;

	node_handle.getParam("/robot/name",robotName);
	ROS_INFO("Init simple_agent_monitor");
	ROS_INFO("Robot name is %s",robotName.c_str());

	DataReader data_reader(node_handle); //reads entities data

	AgentMonitors agent_monitors(robotName); //monitors on entities data

	ros::Publisher factPublisher=node_handle.advertise<situation_assessment_msgs::FactList>("situation_assessment/agent_fact_list",1000);
	ros::Publisher areaPublisher=node_handle.advertise<situation_assessment_msgs::AreaList>("situation_assessment/area_polygons",1000);

	ros::ServiceServer add_area_server=node_handle.advertiseService("situation_assessment/add_area",addArea);
	ros::ServiceServer remove_area_server=node_handle.advertiseService("situation_assessment/remove_area",removeArea);


	ros::ServiceClient add_database_client=node_handle.serviceClient<situation_assessment_msgs::DatabaseRequest>("situation_assessment/add_facts");
	ros::ServiceClient remove_database_client=node_handle.serviceClient<situation_assessment_msgs::DatabaseRequest>("situation_assessment/remove_facts");

	ROS_INFO("Waiting for database to be up");
	add_database_client.waitForExistence();
	remove_database_client.waitForExistence();

	ROS_INFO("Advertising topics and services");

	ros::Rate rate(3);
	ROS_INFO("Starting computation");

	PairMap entity_distances;

	vector<situation_assessment_msgs::Fact> old_fact_list;

	while (ros::ok()) {
		ros::spinOnce();

		//gets entities data
		EntityMap agent_poses=data_reader.getAgentPoses();
		Entity robot_poses=data_reader.getRobotPoses();
		EntityMap object_poses=data_reader.getObjectPoses();
		EntityMap group_poses=data_reader.getGroupPoses();
		StringVectorMap group_members=data_reader.getAgentGroups();

		// if (agent_poses.size()>0) {
			//create some containers with only humans, human and robot, or human robot and objects.
			EntityMap all_agents=agent_poses;
			all_agents[robotName]=robot_poses;
			EntityMap all_entities=all_agents;
			if (object_poses.size()>0) {
				all_entities.insert(object_poses.begin(),object_poses.end());
			}
			updateEntityAreas(all_agents); //update areas linked to entities

			//get facts
			vector<situation_assessment_msgs::Fact> distances=agent_monitors.getDistances(all_agents,all_entities,&entity_distances);
			vector<situation_assessment_msgs::Fact> isMoving=agent_monitors.getIsMoving(all_agents);
			vector<situation_assessment_msgs::Fact> delta_distance=agent_monitors.getDeltaDistances(all_agents,all_entities,entity_distances);
			vector<situation_assessment_msgs::Fact> isInArea=agent_monitors.getIsInArea(all_agents,areas);
			vector<situation_assessment_msgs::Fact> group_contains=agent_monitors.getGroupContains(group_members);	
			vector<situation_assessment_msgs::Fact> object_types=agent_monitors.getEntityType(object_poses);

			situation_assessment_msgs::FactList factList;
			factList.fact_list=group_contains;
			if (isMoving.size()>0) {
				factList.fact_list.insert(factList.fact_list.end(),isMoving.begin(),isMoving.end());
			}
			if (distances.size()>0) {
				factList.fact_list.insert(factList.fact_list.end(),distances.begin(),distances.end());
			}
			if (delta_distance.size()>0) {
				factList.fact_list.insert(factList.fact_list.end(),delta_distance.begin(),delta_distance.end());
			}
			if (isInArea.size()>0) {
				factList.fact_list.insert(factList.fact_list.end(),isInArea.begin(),isInArea.end());
			}
			if (object_types.size()>0) {
				factList.fact_list.insert(factList.fact_list.end(),object_types.begin(),object_types.end());

			}

			factPublisher.publish(factList);

			//publish area list for visualization
			situation_assessment_msgs::AreaList msg_area;
			msg_area.header.stamp=ros::Time::now();
			msg_area.header.frame_id="map";
			vector<geometry_msgs::Polygon> polygon_list;

			for (GeometryPolygonMap::iterator i=msg_areas_map.begin();i!=msg_areas_map.end();i++) {
				polygon_list.push_back(i->second);
			}
			msg_area.areas=polygon_list;
			areaPublisher.publish(msg_area);

			//update the db
			updateDatabase(&add_database_client,&remove_database_client,factList.fact_list,old_fact_list);

			old_fact_list=factList.fact_list;
	 	// }
		rate.sleep();
	}
	ros::shutdown();

}

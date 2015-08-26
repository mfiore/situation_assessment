//ros stuff
#include <ros/ros.h>
#include <tf/transform_listener.h>

//msgs
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Polygon.h>

#include "situation_assessment_msgs/Fact.h"
#include "situation_assessment_msgs/FactList.h"
#include <situation_assessment_msgs/AgentPoseList.h>
#include <situation_assessment_msgs/AgentPose.h>

#include <spencer_tracking_msgs/TrackedPersons.h>
#include <spencer_tracking_msgs/TrackedPerson.h>
#include <spencer_tracking_msgs/TrackedGroups.h>
#include <spencer_tracking_msgs/TrackedGroup.h>
//services
#include <situation_assessment_msgs/AddArea.h>

// //boost
#include <boost/polygon/polygon.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp> 
#include <boost/lexical_cast.hpp> 




//other
#include <utility>
#include <string>
#include <vector>
#include <map>

#include "simple_agent_monitor/RingBuffer.h"
#include "simple_agent_monitor/MathFunctions.h"


using namespace std;
namespace gtl = boost::polygon;

typedef gtl::polygon_data<double> Polygon;
typedef gtl::polygon_traits<Polygon>::point_type Point;

//parameters
string robotName;
vector<string> agents;
vector<string> objects;
double angleThreshold;

//containers
map<string,RingBuffer<geometry_msgs::Pose>> agentPose;
map<string,RingBuffer<geometry_msgs::Pose>> groupPose;
map<string,vector<string> > agents_groups;


map<string,geometry_msgs::Pose> objectPose;
map<pair<string,string>,RingBuffer<double>> agentDistances;
map<string,vector<string>> groups;
 // map<string,bg::model::polygon<bg::model::d2::point_xy<double> > > areas;
map<string,Polygon> areas; 

ros::Publisher agentPosePublisher;

bool use_spencer_tracking;

boost::mutex mutex_agents;;
boost::mutex mutex_groups;;


//sets the pose of an agent into a ringbuffer
void setAgentPoseTf(tf::StampedTransform transform, string name) {
	geometry_msgs::Pose pose;
	pose.position.x=transform.getOrigin().getX();
	pose.position.y=transform.getOrigin().getY();
	pose.position.z=transform.getOrigin().getZ();
	quaternionTFToMsg(transform.getRotation(),pose.orientation);
	if (agentPose[name].size==0) {
		agentPose[name].allocate(10);
	}

	agentPose[name].insert(pose);
}//sets the pose of an agent into a ringbuffer
void setGroupPoseTf(tf::StampedTransform transform, string name) {
	geometry_msgs::Pose pose;
	pose.position.x=transform.getOrigin().getX();
	pose.position.y=transform.getOrigin().getY();
	pose.position.z=transform.getOrigin().getZ();
	quaternionTFToMsg(transform.getRotation(),pose.orientation);
	if (groupPose[name].size==0) {
		groupPose[name].allocate(10);
	}

	groupPose[name].insert(pose);
}

//sets the pose of an agent into a ringbuffer
void setAgentPose(geometry_msgs::Pose pose, string name) {
	if (agentPose[name].size==0) {
		agentPose[name].allocate(10);
	}
	agentPose[name].insert(pose);
}

void setGroupPose(geometry_msgs::Pose pose, string name) {
	if (groupPose[name].size==0) {
		groupPose[name].allocate(10);
	}

	groupPose[name].insert(pose);
}

//gets the poses of all agents and objects from tf
void getPoses() {
	tf::TransformListener listener;
	//get human agents
	geometry_msgs::Pose new_group_pose;
	new_group_pose.position.x=0;
	new_group_pose.position.y=0;
	new_group_pose.position.z=0;

	for (string agentName:agents) {
		try {
			tf::StampedTransform transform;
			listener.waitForTransform("map", agentName, ros::Time(0), ros::Duration(1.0) );
			if (!ros::ok()) {
				return;
			}
			listener.lookupTransform("map",agentName,ros::Time(0),transform);

			setAgentPoseTf(transform,agentName);
			new_group_pose.position.x=new_group_pose.position.x+transform.getOrigin().getX();
			new_group_pose.position.y=new_group_pose.position.y+transform.getOrigin().getY();
			new_group_pose.position.z=new_group_pose.position.z+transform.getOrigin().getZ();

		}     catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
		}
	}

	new_group_pose.position.x=new_group_pose.position.x/agents.size();
	new_group_pose.position.y=new_group_pose.position.y/agents.size();
	new_group_pose.position.z=new_group_pose.position.z/agents.size();
	groupPose["GROUP_1"].insert(new_group_pose);

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
		 setAgentPoseTf(transform,robotName);
	} catch(tf::TransformException ex) {
		ROS_ERROR("%s",ex.what());
	}

}

//publishes the poses of agents as facts
void publishPose() {
	situation_assessment_msgs::AgentPoseList list;
	for (auto agent: agentPose) {
		situation_assessment_msgs::AgentPose pose;
		pose.name=agent.first;
		pose.pose=agent.second.getSequence(1)[0];
		list.agentPoses.push_back(pose);
	}
	agentPosePublisher.publish(list);

}

//calculates the distances between agents and agents and objects
vector<situation_assessment_msgs::Fact> getDistances() {
	boost::lock_guard<boost::mutex> lock(mutex_agents);
	boost::lock_guard<boost::mutex> lock2(mutex_groups);

	vector<situation_assessment_msgs::Fact> result;

	map<string,RingBuffer<geometry_msgs::Pose>> all_poses(agentPose);
	all_poses.insert(groupPose.begin(),groupPose.end());

	for (auto agent1:all_poses) {
		geometry_msgs::Pose pose1,pose2;
		pose1=agent1.second.getSequence(1)[0];
		for (auto agent2:all_poses) {

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
				// cout<<"Distance "<<agent1.first<<" "<<agent2.first<<" "<<d<<"\n";
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

//calculates variations of distances
vector<situation_assessment_msgs::Fact> getDeltaDistances() {
	boost::lock_guard<boost::mutex> lock(mutex_agents);
	boost::lock_guard<boost::mutex> lock2(mutex_groups);

	vector<situation_assessment_msgs::Fact> result;

	map<string,RingBuffer<geometry_msgs::Pose>> all_poses(agentPose);
	all_poses.insert(groupPose.begin(),groupPose.end());

	for (auto agent1:all_poses) {
		geometry_msgs::Pose pose1,pose2;
		pose1=agent1.second.getSequence(1)[0];
		for (auto agent2:all_poses) {
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


//calculate if agents are moving
vector<situation_assessment_msgs::Fact> getIsMoving() {
	boost::lock_guard<boost::mutex> lock(mutex_agents);
	boost::lock_guard<boost::mutex> lock2(mutex_groups);

	vector<situation_assessment_msgs::Fact> result;

	map<string,RingBuffer<geometry_msgs::Pose>> all_poses(agentPose);
	all_poses.insert(groupPose.begin(),groupPose.end());

	for (auto agent1:all_poses) {
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

	}
	return result;
}

vector<situation_assessment_msgs::Fact> getIsFacing() {
	boost::lock_guard<boost::mutex> lock(mutex_agents);
	boost::lock_guard<boost::mutex> lock2(mutex_groups);

	vector<situation_assessment_msgs::Fact> result;

	map<string,RingBuffer<geometry_msgs::Pose>> all_poses(agentPose);
	all_poses.insert(groupPose.begin(),groupPose.end());

	for (auto agent1:all_poses) {
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

//service to add a geometrical area in the environment
bool addArea(situation_assessment_msgs::AddArea::Request &req, situation_assessment_msgs::AddArea::Response &res) {
	using namespace boost::polygon::operators;
	geometry_msgs::Polygon area=req.area;

	// bg::model::polygon<bg::model::d2::point_xy<double> > new_area;

	// for(geometry_msgs::Point32 point:area.points) {
 //        bg::model::d2::point_xy<double> p(point.x, point.y);
 //        boost::geometry::append(new_area, p);
	// }


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
	res.result=true;
	return true;
}

// calculates if agents are present in areas
vector<situation_assessment_msgs::Fact> getIsInArea() {
	boost::lock_guard<boost::mutex> lock(mutex_agents);
	boost::lock_guard<boost::mutex> lock2(mutex_groups);

	vector<situation_assessment_msgs::Fact> result;

	map<string,RingBuffer<geometry_msgs::Pose>> all_poses(agentPose);
	all_poses.insert(groupPose.begin(),groupPose.end());

	for (auto agent:all_poses) {
		map<string,RingBuffer<geometry_msgs::Pose>> agentPose;
		geometry_msgs::Pose pose=agent.second.getSequence(1)[0];


		//boost::tuple<double, double> p = boost::make_tuple(pose.position.x, pose.position.y);
		// bg::model::d2::point_xy<double>  p(pose.position.x,pose.position.y);
		Point p=gtl::construct<Point>(pose.position.x, pose.position.y);
		for (auto an_area:areas) {
			// if (bg::within(p,an_area.second)) {

			if(gtl::contains(an_area.second, p )){
				situation_assessment_msgs::Fact new_fact;
				new_fact.model=robotName;
				new_fact.subject=agent.first;
				new_fact.predicate.push_back("isInArea");
				new_fact.value=an_area.first;

				result.push_back(new_fact);				
			}
		}

		
	}
	return result;

}


void trackedPersonsCallback(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg) {
	boost::lock_guard<boost::mutex> lock(mutex_agents);

	map<string,bool> still_present;

	for (auto agent:agentPose) {
		still_present[agent.first]=false;
	}

	// ROS_INFO("Number of persons %d",msg->tracks.size());
	for (auto person:msg->tracks) {

		string name=boost::lexical_cast<string>(person.track_id);
		still_present[name]=true;
		setAgentPose(person.pose.pose,name);


	}
	for (auto agent:still_present) {

		if (agent.second==false) {
			// ROS_INFO("Agent is not present");
			// ROS_INFO("Old agent pose size %d",agentPose.size());
			agentPose.erase(agent.first);
			// ROS_INFO("New agent pose size %d",agentPose.size());

		}
	}
}
void trackedGroupsCallback(const spencer_tracking_msgs::TrackedGroups::ConstPtr& msg) {
		boost::lock_guard<boost::mutex> lock(mutex_groups);

		map<string,bool> still_present;

		for (auto group:groupPose) {
			still_present[group.first]=false;
		}

		for (auto group:msg->groups) {
			string group_name=boost::lexical_cast<string>(group.group_id);

			still_present[group_name]=true;
			setGroupPose(group.centerOfGravity.pose,group_name);

			vector<string> tracks_in_group;
			for (int a_track:group.track_ids) {
				tracks_in_group.push_back(boost::lexical_cast<string>(a_track));
			}
			agents_groups[group_name]=tracks_in_group;
		}
		for (auto group:still_present) {
			if (group.second==false) {
				groupPose.erase(group.first);
				agents_groups.erase(group.first);
			}
		}
	}

vector<situation_assessment_msgs::Fact> getGroupContains() {
	boost::lock_guard<boost::mutex> lock(mutex_groups);


	vector<situation_assessment_msgs::Fact> result;
	for (auto assignment:agents_groups) {
		for (string agent:assignment.second) {
			situation_assessment_msgs::Fact f;
			f.subject=assignment.first;
			f.predicate.push_back("contains");
			f.value=agent;
			result.push_back(f);
		}
	} 
	for (auto agent:agentPose) {
		situation_assessment_msgs::Fact f;

		f.subject=agent.first;
		f.predicate.push_back("contains");
		f.value=agent.first;
		result.push_back(f);
	}
	return result;
}	



int main(int argc, char** argv) {


	ros::init(argc,argv,"simple_agent_monitor");


	ros::NodeHandle n;

	n.getParam("/robot/name",robotName);
	n.getParam("/human_agents/agent_names",agents);
	n.getParam("/objects",objects);
	n.getParam("/situation_assessment/orientation_angle_threshold",angleThreshold);
	n.getParam("/situation_assessment/use_spencer_tracking",use_spencer_tracking);

	ROS_INFO("Init simple_agent_monitor");
	ROS_INFO("Robot name is %s",robotName.c_str());
	ROS_INFO("Agent names are:");
	for (string agent:agents) {
		ROS_INFO("- %s",agent.c_str());
	}
	ROS_INFO("Object names are:");
	for (string object:objects) {
		ROS_INFO("- %s",object.c_str());
	}
	ROS_INFO("Threshold for orientation is %f",angleThreshold);


	agentPosePublisher=n.advertise<situation_assessment_msgs::AgentPoseList>("situation_assessment/agent_pose_list",1000);
	ros::Publisher factPublisher=n.advertise<situation_assessment_msgs::FactList>("situation_assessment/agent_fact_list",1000);
	ros::ServiceServer add_area_server=n.advertiseService("situation_assessment/add_area",addArea);

	ROS_INFO("Advertising topics and services");

	ros::Subscriber tracked_persons_sub, tracked_groups_sub;
	if (use_spencer_tracking) {
		ROS_INFO("Using spencer tracking. Subscribing to topics");
		tracked_persons_sub=n.subscribe("/spencer/perception/tracked_persons",1000,
			trackedPersonsCallback);
		ROS_INFO("Waiting for tracked persons to be published");
		ros::Rate r(3);
		while (tracked_persons_sub.getNumPublishers()==0 && ros::ok()) {
			r.sleep();
		}

		tracked_groups_sub=n.subscribe("/spencer/perception/tracked_groups",1000,trackedGroupsCallback);
		ROS_INFO("Waiting for tracked groups to be published");
		// while (tracked_groups_sub.getNumPublishers()==0 && ros::ok()) {
		// 	r.sleep();
		// }		
	}
	else {
		ROS_INFO("Not using spencer tracking. Creating fake GROUP_1 with all input agents");
		for (string agentName:agents) {
				groups["GROUP_1"].push_back(agentName);
			}
		groupPose["GROUP_1"].allocate(10);
		agents_groups["GROUP_1"]=agents;
	}

	ros::Rate rate(10.0);
	ROS_INFO("Starting computation");
	while (ros::ok()) {
		if (!use_spencer_tracking) {
			getPoses();
		}
		ros::spinOnce();

		publishPose();

		vector<situation_assessment_msgs::Fact> group_contains=getGroupContains();	
		vector<situation_assessment_msgs::Fact> distances=getDistances();
		vector<situation_assessment_msgs::Fact> isMoving=getIsMoving();
		vector<situation_assessment_msgs::Fact> delta_distance=getDeltaDistances();
		vector<situation_assessment_msgs::Fact> isInArea=getIsInArea();


		situation_assessment_msgs::FactList factList;
		factList.fact_list=group_contains;
		factList.fact_list.insert(factList.fact_list.end(),isMoving.begin(),isMoving.end());
		factList.fact_list.insert(factList.fact_list.end(),distances.begin(),distances.end());
		factList.fact_list.insert(factList.fact_list.end(),delta_distance.begin(),delta_distance.end());
		factList.fact_list.insert(factList.fact_list.end(),isInArea.begin(),isInArea.end());
		// factList.fact_list=isInArea;

		factPublisher.publish(factList);

		rate.sleep();
	}
	ros::shutdown();

}

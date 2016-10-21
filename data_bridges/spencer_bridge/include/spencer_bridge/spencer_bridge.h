/**
spencer_bridge.h

author Michelangelo Fiore.

Data Bridge used for the spencer proejct
*/

#ifndef SPENCER_BRIDGE_H
#define SPENCER_BRIDGE_H


#include <ros/ros.h>
#include <data_lib/data_lib.h>

#include <ros/ros.h>

#include <vector>
#include <map>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/trim.hpp>


#include <situation_assessment_msgs/NamedPoseList.h>
#include <situation_assessment_msgs/NamedPose.h>
#include <situation_assessment_msgs/Group.h>
#include <situation_assessment_msgs/GroupList.h>

#include <situation_assessment_msgs/AddArea.h>
#include <situation_assessment_msgs/NameRequest.h>

#include <spencer_tracking_msgs/TrackedPersons.h>
#include <spencer_tracking_msgs/TrackedPerson.h>
#include <spencer_tracking_msgs/TrackedGroups.h>
#include <spencer_tracking_msgs/TrackedGroup.h>
// #include <spencer_mapping_msgs/Annotations.h>
// #include <spencer_mapping_msgs/Annotation.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>


#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <tinyxml2/tinyxml2.h>

using namespace std;

typedef map<string,geometry_msgs::Pose> PoseMap;
typedef map<string,string> StringMap;
typedef map<string,vector<string> > StringVectorMap;


class SpencerBridge:public DataLib {
public:
	SpencerBridge(ros::NodeHandle node_handle);

private:
	//get data from persons
	void trackedPersonsCallback(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg);
	//get data from groups
	void trackedGroupsCallback(const spencer_tracking_msgs::TrackedGroups::ConstPtr& msg);

	//read object positions from the xml file of the annotated map
	void readObjects();

	geometry_msgs::Point32 rotatePoint(geometry_msgs::Point32 p, geometry_msgs::Point32 pivot, double theta);
	//creates monitor areas linked to the information screens
	void addInformationScreenArea(string name, double x, double y, double theta);
	geometry_msgs::Pose addGateArea(string name, double x, double y, double theta);
	void addOtherArea(string name, vector<geometry_msgs::Pose> vertex_poses);

	geometry_msgs::Pose transformPose(geometry_msgs::Pose init_pose, string frame_id, tf::StampedTransform *transform);


	tf::TransformListener listener_;

	//area informations for the objects to onitor
	double triangle_b_, triangle_h_;
	double gate_b_ , gate_h_;

	boost::mutex mutex_agents_, mutex_objects_,mutex_groups_,mutex_robot_;

	ros::ServiceClient add_area_client_;

	ros::Subscriber tracked_persons_sub_, tracked_groups_sub_;

	//paths to the annotated map xml documents
	string class_document_path_;
	string main_doc_name_;
	string main_doc_path_;
};

#endif
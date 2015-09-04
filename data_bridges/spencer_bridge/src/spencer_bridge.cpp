#include <spencer_bridge/spencer_bridge.h>


SpencerBridge::SpencerBridge(ros::NodeHandle node_handle):DataLib("spencer_bridge",node_handle) {
	
	node_handle_.getParam("situation_assessment/triangle_b",triangle_b_);
	node_handle_.getParam("situation_assessment/triangle_h",triangle_h_);
	node_handle_.getParam("supervision/doc_path",main_doc_path_);
	node_handle_.getParam("supervision/doc_name",main_doc_name_);

	ROS_INFO("Main document path and name are %s %s",main_doc_path_.c_str(),main_doc_name_.c_str());
	
	ROS_INFO("Monitor area for screens has h %f and b %f",triangle_h_,triangle_b_);
	add_area_client_=node_handle_.serviceClient<situation_assessment_msgs::AddArea>("situation_assessment/add_area");
	remove_area_client_=node_handle_.serviceClient<situation_assessment_msgs::NameRequest>("situation_assessment/remove_area");
	
	ROS_INFO("Waiting for area servers to be available");
	add_area_client_.waitForExistence();
	remove_area_client_.waitForExistence();

	ros::Rate r(3);

	if (track_agents_) {
		tracked_persons_sub_=node_handle_.subscribe("/spencer/perception/tracked_persons",1000, 
			&SpencerBridge::trackedPersonsCallback,this);
		ROS_INFO("Waiting for tracked persons to be published");
		while (tracked_persons_sub_.getNumPublishers()==0 && ros::ok()) {
			r.sleep();
		}
	}
	if (track_objects_) {
		readObjects();
	}
	if (track_groups_) {
		tracked_groups_sub_=node_handle_.subscribe("/spencer/perception/tracked_groups",1000,
		&SpencerBridge::trackedGroupsCallback,this);
		ROS_INFO("Waiting for tracked groups to be published");
	}	

}


void SpencerBridge::trackedPersonsCallback(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg) {
	boost::lock_guard<boost::mutex> lock(mutex_agents_);

	map<string,bool> still_present;

	for (EntityMap::iterator agent=agent_poses_.begin();agent!=agent_poses_.end();agent++) {
		still_present[agent->first]=false;
	}

	for (int i=0; i<msg->tracks.size();i++) {

		string name=boost::lexical_cast<string>(msg->tracks[i].track_id);
		still_present[name]=true;
		agent_poses_[name].pose=msg->tracks[i].pose.pose;
		agent_poses_[name].name=name;
		agent_poses_[name].type="HUMAN";
	}
	for (map<string,bool>::iterator agent=still_present.begin(); agent!=still_present.end();
		agent++) {
		if (agent->second==false) {
			agent_poses_.erase(agent->first);
		}
	}
}

void SpencerBridge::trackedGroupsCallback(const spencer_tracking_msgs::TrackedGroups::ConstPtr& msg) {
		boost::lock_guard<boost::mutex> lock(mutex_groups_);

		map<string,bool> still_present;

		for (EntityMap::iterator group=group_poses_.begin();group!=group_poses_.end();group++) {
			still_present[group->first]=false;
		}

		for (int i=0; i<msg->groups.size();i++) {
			string group_name=boost::lexical_cast<string>(msg->groups[i].group_id);

			still_present[group_name]=true;
			group_poses_[group_name].pose=msg->groups[i].centerOfGravity.pose;
			group_poses_[group_name].name=group_name;
			group_poses_[group_name].type="GROUP";

			vector<string> tracks_in_group;
			BOOST_FOREACH(int a_track,msg->groups[i].track_ids) {
				tracks_in_group.push_back(boost::lexical_cast<string>(a_track));
			}
			agent_groups_[group_name]=tracks_in_group;
		}
		for (map<string,bool>::iterator group=still_present.begin(); group!=still_present.end();
			group++) {
			if (group->second==false) {
				group_poses_.erase(group->first);
				agent_groups_.erase(group->first);
			}
		}
	}


void SpencerBridge::removeArea(string name ) {
	situation_assessment_msgs::NameRequest remove_area_request;
	remove_area_request.request.name=name;

	if (remove_area_client_.call(remove_area_request)) {
		ROS_INFO("Removed area %s",name.c_str());
	}
	else {
		ROS_WARN("Failed to remove area");
	}
}

geometry_msgs::Point32 SpencerBridge::rotatePoint(geometry_msgs::Point32 p, geometry_msgs::Point32 pivot, double theta) {
	// float s = sin(angle);
	// float c = cos(angle);

	// // translate point back to origin:
	// p.x -= cx;
	// p.y -= cy;

	// // rotate point
	// float xnew = p.x * c - p.y * s;
	// float ynew = p.x * s + p.y * c;

	// // translate point back:
	// p.x = xnew + cx;
	// p.y = ynew + cy;

	p.x=p.x-pivot.x;
	p.y=p.y-pivot.y;

	geometry_msgs::Point32 rotated_p;

	rotated_p.x=p.x*cos(theta)-p.y*sin(theta);
	rotated_p.y=p.x*sin(theta)+p.y*cos(theta);

	rotated_p.x=rotated_p.x+pivot.x;
	rotated_p.y=rotated_p.y+pivot.y;

	return rotated_p;
}

void SpencerBridge::addInformationScreenArea(string name, double x, double y, double theta) {
	// add triangle area and name of the screen
	geometry_msgs::Polygon area_polygon;
	vector<geometry_msgs::Point32> points;

	geometry_msgs::Point32 p1,p2,p3;
	p1.x=x;
	p1.y=y;

	p2.x=p1.x-triangle_b_/2;
	p2.y=p1.y+triangle_h_;
	ROS_INFO("%f %f",p2.x,p2.y);
	p2=rotatePoint(p2,p1,6.28);
	ROS_INFO("%f %f",p2.x,p2.y);

	p3.x=p1.x+triangle_b_/2;
	p3.y=p1.y+triangle_h_;
	ROS_INFO("%f %f",p3.x,p3.y);
	p3=rotatePoint(p3,p1,6.28);
	ROS_INFO("%f %f",p3.x,p3.y);


	points.push_back(p1);
	points.push_back(p2);
	points.push_back(p3);

	area_polygon.points=points;

	//now we add an area corresponding to the triangle, with the name of the object
	//and also we add the object to monitoring

	situation_assessment_msgs::AddArea add_area_request;
	add_area_request.request.name=name;
	add_area_request.request.area=area_polygon;

	if (add_area_client_.call(add_area_request)) {
		ROS_INFO("Monitoring %s",name.c_str());
	}
	else {
		ROS_WARN("Failed to monitor area");
	}
}

void SpencerBridge::readObjects() {
	boost::lock_guard<boost::mutex> lock(mutex_objects_);

	ROS_INFO("Reading object information");


	tinyxml2::XMLDocument mainDoc,classDoc;
	string full_main_doc_path=main_doc_path_+main_doc_name_+".xml";
	class_document_path_=main_doc_path_+main_doc_name_+"_classes.xml";
	int error1=mainDoc.LoadFile( full_main_doc_path.c_str() );
	int error2=classDoc.LoadFile(class_document_path_.c_str());

	if (error1!=0) {
		ROS_ERROR("Cannot find main map file");
		return;
	}
	if (error2!=0) {
		ROS_INFO("Cannot find class file");
		return;
	}
	else {
		ROS_INFO("Found docs. Starting parsing");
	}
	string resolutionString=mainDoc.FirstChildElement("project")->FirstChildElement("map")->FirstChildElement("resolution")->GetText();
	boost::trim(resolutionString);
	double resolution=boost::lexical_cast<double>(resolutionString);
	cout<<"resolution is "<<resolution<<"\n";

	string x_string=mainDoc.FirstChildElement("project")->FirstChildElement("map")->FirstChildElement("origin")->FirstChildElement("x")->GetText();
	string y_string=mainDoc.FirstChildElement("project")->FirstChildElement("map")->FirstChildElement("origin")->FirstChildElement("x")->GetText();

	boost::trim(x_string);
	boost::trim(y_string);

	double map_origin_x=boost::lexical_cast<double>(x_string);
	double map_origin_y=boost::lexical_cast<double>(y_string);

	tinyxml2::XMLNode *classNode=classDoc.FirstChildElement("classes")->FirstChildElement();
	while (classNode!=NULL) {
		ROS_INFO("In class node");

		string class_name=classNode->FirstChildElement("name")->GetText();

		tinyxml2::XMLNode *annotation_node=classNode->FirstChildElement("annotations")->FirstChild();

		int i=0;
		while (annotation_node!=NULL) {
			ROS_INFO("In annotation node");
			i++;
			geometry_msgs::Pose pose;

			string scx=annotation_node->FirstChildElement("center")->FirstChildElement("x")->GetText();
			string scy=annotation_node->FirstChildElement("center")->FirstChildElement("y")->GetText();
			boost::trim(scx);
			boost::trim(scy);

			double cx=boost::lexical_cast<double>(scx);
			double cy=boost::lexical_cast<double>(scy);
			
			string stheta=annotation_node->FirstChildElement("orientation")->GetText();
			boost::trim(stheta);
			double theta=boost::lexical_cast<double>(stheta);

			pose.position.x=80-(cx*resolution+map_origin_x);
			pose.position.y=80-(cy*resolution+map_origin_y);

			tf::Quaternion quaternion;
			quaternion.setRPY(0,0,theta);
			tf::quaternionTFToMsg(quaternion,pose.orientation);

			ROS_INFO("Creating entity");
			Entity object;
			object.name=class_name+"_"+boost::lexical_cast<string>(i);;
			object.type=class_name;
			object.pose=pose;
			object_poses_[object.name]=object;

			addInformationScreenArea(object.name,pose.position.x,pose.position.y,theta);
			annotation_node=annotation_node->NextSibling();
		}
		classNode=classNode->NextSibling();

	}
}



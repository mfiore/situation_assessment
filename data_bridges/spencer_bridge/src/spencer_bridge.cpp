#include <spencer_bridge/spencer_bridge.h>


SpencerBridge::SpencerBridge(ros::NodeHandle node_handle):DataLib("spencer_bridge",node_handle) {
	string robot_name;



	node_handle.getParam("situation_assessment/information_screen_area_b",triangle_b_);
	node_handle.getParam("situation_assessment/information_screen_area_h",triangle_h_);
	node_handle.getParam("situation_assessment/gate_area_b",gate_b_);
	node_handle.getParam("situation_assessment/gate_area_h",gate_h_);
	node_handle.getParam("situation_assessment/doc_path",main_doc_path_);
	node_handle.getParam("situation_assessment/doc_name",main_doc_name_);

	node_handle.getParam("/robot/name",robot_name);

	ROS_INFO("SPENCER BRIDGE - Got parameters");
	ROS_INFO("Main document path and name are %s %s",main_doc_path_.c_str(),main_doc_name_.c_str());
	
	ROS_INFO("Monitor area for screens has h %f and b %f",triangle_h_,triangle_b_);
	add_area_client_=node_handle.serviceClient<situation_assessment_msgs::AddArea>("situation_assessment/add_area");
	
	ROS_INFO("Waiting for area servers to be available");
	add_area_client_.waitForExistence();

	ros::Rate r(3);

	if (track_agents_) {
		tracked_persons_sub_=node_handle.subscribe("/spencer/perception/tracked_persons",1, 
			&SpencerBridge::trackedPersonsCallback,this);
		ROS_INFO("Waiting for tracked persons to be published");
		while (tracked_persons_sub_.getNumPublishers()==0 && ros::ok()) {
			r.sleep();
		}
		ROS_INFO("Connected to tracked person");
	}
	if (track_objects_) {
		readObjects();
	}
	if (track_groups_) {
		tracked_groups_sub_=node_handle.subscribe("/spencer/perception/tracked_groups",1,
		&SpencerBridge::trackedGroupsCallback,this);
		ROS_INFO("SPENCER_BRIDGE Waiting for tracked groups to be published");
	}	

	//creates area linked to the robot's position

	// situation_assessment_msgs::AddArea add_area_request;
	// geometry_msgs::Polygon area_polygon;
	// vector<geometry_msgs::Point32> points;

	// geometry_msgs::Point32 p1,p2,p3,p4;
	// p1.x=2;
	// p1.y=0;

	// p2.x=-2;
	// p2.y=0;

	// p3.x=-6;
	// p3.y=-7;

	// p4.x=6;
	// p4.y=-7;

	// points.push_back(p1);
	// points.push_back(p2);
	// points.push_back(p3);
	// points.push_back(p4);

	// add_area_request.request.name=robot_name;
	// add_area_request.request.linked_to_entity=robot_name;
	// area_polygon.points=points;
	// add_area_request.request.area=area_polygon;


	// if (add_area_client_.call(add_area_request)) {
	// 	ROS_INFO("SPENCER_BRIDGE Monitoring %s",robot_name.c_str());
	// }
	// else {
	// 	ROS_WARN("Failed to monitor area");
	// }

}


geometry_msgs::Pose SpencerBridge::transformPose(geometry_msgs::Pose init_pose, string frame_id,
	tf::StampedTransform *transform){

    geometry_msgs::Pose res;

    // ROS_DEBUG("Transform Pose in RRT, in Frame %s", planner_frame_.c_str());

    tf::Pose source;

    tf::Quaternion q= tf::createQuaternionFromRPY(0,0,tf::getYaw(init_pose.orientation));

    tf::Matrix3x3 base(q);

    source.setOrigin(tf::Vector3(init_pose.position.x, init_pose.position.y, 0));

    source.setBasis(base);

    /// Apply the proper transform
    tf::Pose result = *transform*source;


	   res.position.x = result.getOrigin().x() ;
    res.position.y = result.getOrigin().y() ;
    res.position.z = result.getOrigin().z() ;

    tf::quaternionTFToMsg(result.getRotation(), res.orientation);

    // res.header = init_pose.header;
    // res.header.frame_id = planner_frame_;

    return res;

}




void SpencerBridge::trackedPersonsCallback(const spencer_tracking_msgs::TrackedPersons::ConstPtr& msg) {
	boost::lock_guard<boost::mutex> lock(mutex_agents_);

	map<string,bool> still_present;


	for (EntityMap::iterator agent=agent_poses_.begin();agent!=agent_poses_.end();agent++) {
		still_present[agent->first]=false;
	}

	tf::StampedTransform transform;


	// ROS_INFO("SPENCER_BRIDGE msg tracks size %ld",msg->tracks.size());
	if (msg->tracks.size()>0) {
		try{
		    // will transform data in the goal_frame into the planner_frame_
		    listener_.waitForTransform( "/map", msg->header.frame_id, ros::Time(0), ros::Duration(0.50));
		    listener_.lookupTransform(  "/map",  msg->header.frame_id, ros::Time(0), transform);

		}
		catch(tf::TransformException ex){
		    ROS_ERROR("SPENCER_BRIDGE %s",ex.what());
			return;
		}
	}
	for (int i=0; i<msg->tracks.size();i++) {

		string name=boost::lexical_cast<string>(msg->tracks[i].track_id);


		geometry_msgs::Pose pose=transformPose(msg->tracks[i].pose.pose,"odom",&transform);


		still_present[name]=true;
		agent_poses_[name].pose=pose;
		agent_poses_[name].name=name;
		agent_poses_[name].type="human";
		agent_poses_[name].category="agent";
		// ROS_INFO("Got new agent pose %s %f %f",name.c_str(),pose.position.x,pose.position.y);
	}
	// // ROS_INFO("SPENCER_BRIDGE Finished, checking presence");
	for (map<string,bool>::iterator agent=still_present.begin(); agent!=still_present.end();
		agent++) {
		if (agent->second==false) {
			// ROS_INFO("SPENCER_BRIDGE Erasing");
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
			group_poses_[group_name].type="group";
			group_poses_[group_name].category="group";

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




geometry_msgs::Point32 SpencerBridge::rotatePoint(geometry_msgs::Point32 p, geometry_msgs::Point32 pivot, double theta) {

	p.x=p.x-pivot.x;
	p.y=p.y-pivot.y;

	geometry_msgs::Point32 rotated_p;

	rotated_p.x=p.x*cos(theta)-p.y*sin(theta);
	rotated_p.y=p.x*sin(theta)+p.y*cos(theta);

	rotated_p.x=rotated_p.x+pivot.x;
	rotated_p.y=rotated_p.y+pivot.y;

	return rotated_p;
}

geometry_msgs::Pose SpencerBridge::addGateArea(string name, double x, double y, double theta) {
	ROS_INFO("SPENCER_BRIDGE Adding Gate Area");

	ROS_INFO("Theta is %f",theta);
	// add rectangulare area and name of the gate
	geometry_msgs::Polygon area_polygon;
	vector<geometry_msgs::Point32> points;


	geometry_msgs::Point32 center;
	center.x=x;
	center.y=y;

	geometry_msgs::Point32 p1,p2,p3,p4;
	p1.x=x+gate_b_/2;
	p1.y=y;

	p2.x=p1.x;
	p2.y=p1.y+gate_h_;

	p3.x=p2.x-gate_b_;
	p3.y=p2.y;

	p4.x=p3.x;
	p4.y=p3.y-gate_h_;

	// center.x=(p1.x+p2.x+p3.x+p4.x)/4;
	// center.y=(p1.y+p2.y+p3.y+p4.y)/4;
	p1=rotatePoint(p1,center,theta);
	p2=rotatePoint(p2,center,theta);
	p3=rotatePoint(p3,center,theta);
	p4=rotatePoint(p4,center,theta);


	points.push_back(p1);
	points.push_back(p2);
	points.push_back(p3);
	points.push_back(p4);

	area_polygon.points=points;

	geometry_msgs::Pose new_center;
	new_center.position.x=(p1.x+p2.x+p3.x+p4.x)/4;
	new_center.position.y=(p1.y+p2.y+p3.y+p4.y)/4;
	ROS_INFO("new center %f %f",new_center.position.x,new_center.position.y);

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
	return new_center;

}

void SpencerBridge::addInformationScreenArea(string name, double x, double y, double theta) {
	ROS_INFO("SPENCER_BRIDGE Adding Information Screen Area");

	// add triangle area and name of the screen
	geometry_msgs::Polygon area_polygon;
	vector<geometry_msgs::Point32> points;

	geometry_msgs::Point32 p1,p2,p3;
	p1.x=x;
	p1.y=y;

	p2.x=p1.x-triangle_b_/2;
	p2.y=p1.y+triangle_h_;
	ROS_INFO("%f %f",p2.x,p2.y);
	p2=rotatePoint(p2,p1,theta);
	ROS_INFO("%f %f",p2.x,p2.y);

	p3.x=p1.x+triangle_b_/2;
	p3.y=p1.y+triangle_h_;
	ROS_INFO("%f %f",p3.x,p3.y);
	p3=rotatePoint(p3,p1,theta);
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

void SpencerBridge::addOtherArea(string name, vector<geometry_msgs::Pose> vertex_poses) {
	ROS_INFO("SPENCER_BRIDGE Adding Other Area");


	geometry_msgs::Polygon area_polygon;
	vector<geometry_msgs::Point32> points;

	for (int i=0; i<vertex_poses.size();i++) {
		geometry_msgs::Point32 point;	
		point.x=vertex_poses[i].position.x;
		point.y=vertex_poses[i].position.y;
		points.push_back(point);
	}

	area_polygon.points=points;

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
	ROS_INFO("SPENCER_BRIDGE map origin is %f %f",map_origin_x,map_origin_y);

	tinyxml2::XMLNode *classNode=classDoc.FirstChildElement("classes")->FirstChildElement();
	while (classNode!=NULL && ros::ok()) {
		ROS_INFO("In class node");

		string class_name=classNode->FirstChildElement("name")->GetText();

		tinyxml2::XMLNode *annotation_node=classNode->FirstChildElement("annotations")->FirstChild();

		int i=0;
		while (annotation_node!=NULL && ros::ok()) {
			i++;
			geometry_msgs::Pose center_pose;

			tinyxml2::XMLElement *child_node=annotation_node->FirstChildElement();

			vector<geometry_msgs::Pose> vertex_poses;
			Entity new_object;
			double theta=0;

			while (child_node!=NULL) {
				string child_name=child_node->Name();
				if (child_name=="vertex") {

					string scx=child_node->FirstChildElement("x")->GetText();
					string scy=child_node->FirstChildElement("y")->GetText();
					boost::trim(scx);
					boost::trim(scy);

					geometry_msgs::Pose this_pose;
					this_pose.position.x=(boost::lexical_cast<double>(scx)*resolution+map_origin_x);
					this_pose.position.y=-1*(boost::lexical_cast<double>(scy)*resolution+map_origin_y);
					vertex_poses.push_back(this_pose);
				}
				else if (child_name=="name") {
					new_object.name=child_node->GetText();
				}
				else if (child_name=="orientation") {
					string stheta=child_node->GetText();
					boost::trim(stheta);
					theta=boost::lexical_cast<double>(stheta);
				}
				child_node=child_node->NextSiblingElement();
			}
			if (vertex_poses.size()==0) ROS_ERROR("No vertex found");
			center_pose=vertex_poses[0];
            ROS_INFO("Vertex pose: %f, %f",vertex_poses[0].position.x, vertex_poses[0].position.y);
			for (int i=1;i<vertex_poses.size();i++) {
				ROS_INFO("Vertex pose: %f, %f",vertex_poses[i].position.x, vertex_poses[i].position.y);
				double new_center_x=center_pose.position.x+vertex_poses[i].position.x;
				double new_center_y=center_pose.position.y+vertex_poses[i].position.y;
				center_pose.position.x=new_center_x;
				center_pose.position.y=new_center_y;
			}
			center_pose.position.x=center_pose.position.x/vertex_poses.size();
			center_pose.position.y=center_pose.position.y/vertex_poses.size();
            ROS_INFO("Center %f %f", center_pose.position.x, center_pose.position.y);
			tf::Quaternion quaternion;
			quaternion.setRPY(0,0,theta);
			tf::quaternionTFToMsg(quaternion,center_pose.orientation);

			new_object.category="object";
			new_object.type=class_name;
			new_object.pose=center_pose;

			if (class_name=="Information_Screen") {
				addInformationScreenArea(new_object.name,center_pose.position.x,center_pose.position.y,theta+3.14);
			}
			else if (class_name=="Gate") {
				new_object.pose=addGateArea(new_object.name,center_pose.position.x,center_pose.position.y,theta+3.14);
				new_object.pose.orientation.w=1;
			}
			else {
				addOtherArea(new_object.name,vertex_poses);
			}
			object_poses_[new_object.name]=new_object;

			annotation_node=annotation_node->NextSibling();
		}
		if (!ros::ok()) return;
		classNode=classNode->NextSibling();

	}
}



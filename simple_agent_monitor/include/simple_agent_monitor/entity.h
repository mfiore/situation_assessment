#ifndef ENTITY_H
#define ENTITY_H

#include <string>
#include <simple_agent_monitor/ring_buffer.h>
#include <geometry_msgs/Pose.h>

struct Entity {
	string name;
	RingBuffer<geometry_msgs::Pose> pose;
	string type;
};

#endif
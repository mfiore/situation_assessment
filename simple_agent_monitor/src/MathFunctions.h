/*
 * MathFunctions.cpp
 *
 *  Created on: Jun 3, 2015
 *      Author: mfiore
 */



#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>


#include <math.h>
using namespace std;


double calculateDistance(geometry_msgs::Point p1, geometry_msgs::Point p2);


bool isMoving(vector<geometry_msgs::Point> vp);

bool isFacing(geometry_msgs::Pose p1, geometry_msgs::Point p2);

double isInAngle(geometry_msgs::Point p1,geometry_msgs::Point p2, double angleDir, double angleThreshold);
double relativeAngle(geometry_msgs::Point p1,geometry_msgs::Point p2, double angleDir);

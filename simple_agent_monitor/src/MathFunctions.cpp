/*
 * MathFunctions.cpp
 *
 *  Created on: Jun 3, 2015
 *      Author: mfiore
 */



#include "MathFunctions.h"


double calculateDistance(geometry_msgs::Point p1, geometry_msgs::Point p2) {
	return sqrt(pow(p2.x-p1.x,2)+pow(p2.y-p1.y,2));//+pow(p2.z-p1.z,2));
}


bool isMoving(vector<geometry_msgs::Point> vp) {
	 geometry_msgs::Point p1,p2;

	 double sum=0;
	 for (int i=0; i<vp.size()-1;i++){
		 sum=sum+calculateDistance(vp[i],vp[i+1]);
	 }
     cout<<"Is moving sum "<<sum<<"\n";
	 return sum>0.3;
}

bool isFacing(geometry_msgs::Pose p1, geometry_msgs::Point p2) {
	tf::Quaternion quaternion;
	tf::quaternionMsgToTF(p1.orientation,quaternion);
	double yaw=tf::getYaw(quaternion);
    return isInAngle(p1.position, p2, yaw, 0.5);


}
// Return true if ent2 is in the angle from ent1 with direction angleDir and size angleThreshold

double isInAngle(geometry_msgs::Point p1,geometry_msgs::Point p2, double angleDir,
		double angleThreshold) {
    double angleResult = relativeAngle(p1, p2, angleDir);

    if (fabs(angleResult) > angleThreshold) {
        return 0.0;
    } else {
        return (angleThreshold - fabs(angleResult)) / angleThreshold;
    }
}

// Return angle between ent2 and ent1 and angleDir direction

double relativeAngle(geometry_msgs::Point p1,geometry_msgs::Point p2, double angleDir) {
    double actualAngleDir = acos((fabs(p1.x - p2.x)) / calculateDistance(p1,p2));

    // Trigonometric adjustment
    if (p1.x < p2.x)
        actualAngleDir = 3.1416 - actualAngleDir;

    if (p2.y < p1.y)
        actualAngleDir = -actualAngleDir;

    return angleDir - actualAngleDir;
}

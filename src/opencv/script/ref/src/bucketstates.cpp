/*
 * bucketStates.cpp
 *
 *  Created on: 22 Jan, 2014
 *      Author: lynnette & Thien
 */

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <string.h>
#include <queue>

#include "bucketstates.h"

std::queue<Action> history;

//Look for red blob state
LookForRedBlobState::LookForRedBlobState(){
	ROS_INFO("Looking for red blob");
}

boost::shared_ptr<State> LookForRedBlobState::gotFrame (cv::Mat image, RectData rectData){
	return shared_from_this();
}

//Diving state
DiveState::DiveState(double secondsToDive, boost::shared_ptr<State> nextState){
	ROS_INFO("Diving for %lf secs", secondsToDive);
	transitionTime = ros::Time::now().toSec() + secondsToDive;
	this->nextState = nextState;
}

boost::shared_ptr<State> DiveState::gotFrame (cv::Mat image, RectData rectData){
	if (ros::Time::now().toSec() > transitionTime)
		return nextState;

	//Dive
	bbauv_msgs::controller msg;
	msg.depth_setpoint = DEPTH_POINT;
	publishMovement(msg);

	return shared_from_this();
}

//Hover state
HoverState::HoverState(){
	ROS_INFO("Hovering above red bucket");
}

boost::shared_ptr<State> HoverState::gotFrame (cv::Mat image, RectData rectData){
	return shared_from_this();
}

//Drop ball in center state
DropBallState::DropBallState() {
	ROS_INFO("Centering ball to drop");
}

boost::shared_ptr<State> DropBallState::gotFrame (cv::Mat image, RectData rectData){
	return shared_from_this();
}

//Lost bucket state
LostRedBucketState::LostRedBucketState(){
	ROS_INFO("UHOH. Can't find red bucket...");
}

boost::shared_ptr<State> LostRedBucketState::gotFrame (cv::Mat image, RectData rectData){
	return shared_from_this();
}

//Surface State
SurfaceState::SurfaceState(){
	ROS_INFO("Surfacing");
}

boost::shared_ptr<State> SurfaceState::gotFrame (cv::Mat image, RectData rectData){
	return shared_from_this();
}

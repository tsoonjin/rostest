//States for the linefollower

/* 
	linefollowerstates.cpp
	States for line follower
	Date created: 10 Jan 2014
	Author: Lynnette & Thein
*/

#include <string.h>
#include <queue>
#include <stack>

#include "linefollowingstates.h"

std::queue<double> hist; //queue to store previous angles;
std::stack<double> actionsHist; //stack to store previous movements;

// Look for line state
LookForLineState::LookForLineState(LineFollower* fl) {
	ROS_INFO("Looking for line");
	follower = fl;
}

boost::shared_ptr<State> LookForLineState::gotFrame(cv::Mat image, RectData rectData) {
	if (rectData.detected)
		return boost::shared_ptr<State>(new StraightLineState(follower));

	bbauv_msgs::ControllerGoal msg;
	msg.depth_setpoint = DEPTH_POINT;
	//msg.heading_setpoint = rectData.heading + 10;
	follower->publishMovement(msg);
	return shared_from_this();
}

//Temporary State
TemporaryState::TemporaryState(LineFollower* fl,
							   double secondsToReverse,
							   boost::shared_ptr<State> nextState,
							   double speed = -0.6) {
	ROS_INFO("Reversing for %lf secs", secondsToReverse);
	follower = fl;
	transitionTime = ros::Time::now().toSec() + secondsToReverse;
	this->nextState = nextState;
	this->speed = speed;
}

boost::shared_ptr<State> TemporaryState::gotFrame(cv::Mat image, RectData rectData) {
	if (ros::Time::now().toSec() > transitionTime)
		return nextState;

	bbauv_msgs::ControllerGoal msg;
	msg.depth_setpoint = DEPTH_POINT;
	msg.heading_setpoint = rectData.heading;
	msg.forward_setpoint = speed;
	follower->publishMovement(msg);
	return shared_from_this();
}

//Dive State
DiveState::DiveState (LineFollower* fl, double secondsToDive, boost::shared_ptr<State> nextState) {
	ROS_INFO("Diving for %lf secs", secondsToDive);
	follower = fl;
	transitionTime = ros::Time::now().toSec() + secondsToDive;
	this->nextState = nextState;
}

boost::shared_ptr<State> DiveState::gotFrame(cv::Mat image, RectData rectData) {
	if (ros::Time::now().toSec() > transitionTime)
		return nextState;

	//Dive
	bbauv_msgs::ControllerGoal msg;
	msg.depth_setpoint = DEPTH_POINT;
	msg.heading_setpoint = rectData.heading;
	follower->publishMovement(msg);
	return shared_from_this();
}

//Surface State
SurfaceState::SurfaceState (LineFollower* fl, double heading) {
	ROS_INFO("Surfacing");

	follower = fl;
    bbauv_msgs::ControllerGoal msg;
    msg.depth_setpoint = 0.2;
    msg.heading_setpoint = heading;
    follower->publishMovement(msg);
}

boost::shared_ptr<State> SurfaceState::gotFrame(cv::Mat, RectData) {
	return shared_from_this();
}

//Straight Line State
StraightLineState::StraightLineState(LineFollower* fl) {
	ROS_INFO("Following a straight line");
	follower = fl;
}

boost::shared_ptr<State> StraightLineState::gotFrame(cv::Mat image, RectData rectData) {
	if (!rectData.detected) {
		boost::shared_ptr<State> lNextState(new LookForLineState(follower));
		return boost::shared_ptr<State>(new TemporaryState(follower, 0.5, lNextState));
	}

	int screen_width = image.cols;
	int screen_center_x = screen_width / 2;

	double delta_x = (double) (rectData.center.x - screen_center_x) / screen_width;
	ROS_INFO("x-offset: %lf", delta_x);

	bbauv_msgs::ControllerGoal msg;
	msg.depth_setpoint = DEPTH_POINT;

    //if the rect is too far off centre, do aggressive sidemove
	if (abs(delta_x) > 0.3) {
		ROS_INFO("Box too far off centre! Aggressive sidemove");
		msg.heading_setpoint = normHeading(rectData.heading - rectData.angle);
		msg.sidemove_setpoint = delta_x < 0 ? 1.0 : -1.0;
		follower->publishMovement(msg);
		return shared_from_this();
	}

	std::cout << rectData.angle << std::endl;

	//Based on previous angle, determine if the new angle should be pointing the opposite direction
	if (!hist.empty()) {
		double oppAngle = rectData.angle > 0 ? rectData.angle - 180 : rectData.angle + 180;
		if (abs(rectData.angle - hist.back()) > abs(oppAngle - hist.back())) {
			rectData.angle = oppAngle;
		}
	}

	if ((int) hist.size() > 100) hist.pop();
	hist.push(rectData.angle);

	if (delta_x < -x_strip_threshold) {
		msg.sidemove_setpoint = 0.5;
	} else if (delta_x > x_strip_threshold) {
		msg.sidemove_setpoint = -0.5;
	}

	if (abs(rectData.angle) < 10) {
		//Keep moving forward
		msg.heading_setpoint = rectData.heading;
		msg.forward_setpoint = 0.9;
		ROS_INFO("Forward! Heading: %lf, sidemove: %lf", msg.heading_setpoint, msg.sidemove_setpoint);
	} else {
		if (msg.sidemove_setpoint == 0 && abs(rectData.angle > 10)) {
			msg.sidemove_setpoint = rectData.angle / 60 * 0.2;
		}

		double angle_diff = rectData.angle;
		if (angle_diff > 30) {
			angle_diff = rectData.angle > 0 ? 30.0 : -30.0;
		}
		msg.heading_setpoint = normHeading(rectData.heading - angle_diff);
		ROS_INFO("Moving: %lf side, %lf heading", msg.sidemove_setpoint, msg.heading_setpoint);
	}
	follower->publishMovement(msg);
	return shared_from_this();
}

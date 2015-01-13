/*
 * bucketStates.h
 *
 *  Created on: 22 Jan, 2014
 *      Author: Lynnette & Thien
 */

#ifndef BUCKETSTATES_H_
#define BUCKETSTATES_H_

#include <ros/ros.h>
#include <bbauv_msgs/controller.h>

#include <boost/enable_shared_from_this.hpp>

#include <stdlib.h>
#include <string.h>

//Global constants
extern const int DEPTH_POINT;
extern ros::Publisher movementPub;

inline void publishMovement(const bbauv_msgs::controller& movement){
	movementPub.publish(movement);
}

//Structure for bounding box
struct RectData{
	bool detected;
	cv::Point2f center;
	cv::RotatedRect maxRect;
};

//Structure to store previous actions
struct Action{
	double forward;
	double backward;
	double sidemove;
	double heading;
};

//Abstract base class for states
class State : public boost::enable_shared_from_this<State>{
public:
	State() {};
	virtual ~State() {};
	virtual boost::shared_ptr<State> gotFrame(cv::Mat, RectData rectData) = 0;
};

// Looking for a red blob, diameter 50cm
class LookForRedBlobState : public State{
public:
	LookForRedBlobState();
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
};

// Diving in...
class DiveState : public State{
public:
	DiveState (double secondsToDive, boost::shared_ptr<State> nextState);
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
private:
	double transitionTime;
	boost::shared_ptr<State> nextState;
};

//Hover once found red blob
class HoverState : public State{
public:
	HoverState();
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
};

//Drop ball in the center
class DropBallState : public State{
public:
	DropBallState();
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
};

//In case lost red bucket
class LostRedBucketState : public State{
public:
	LostRedBucketState();
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
};

//Surface when done
class SurfaceState : public State{
public:
	SurfaceState();
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
};

#endif /* BUCKETSTATES_H_ */

//Header file for linefollowingstates

/* 
	linefollowerstates.h
	Header file for line following states
	Date created: 10 Jan 2014
	Author: Lynnette & Thien
*/

#ifndef LINEFOLLOWINGSTATES_H_
#define LINEFOLLOWINGSTATES_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "bbauv_msgs/compass_data.h"
#include "bbauv_msgs/controller.h"
#include <bbauv_msgs/ControllerAction.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <boost/enable_shared_from_this.hpp>

#include <stdlib.h>
#include <string.h>

//Global constants
//extern const int endTime;
extern const int DEPTH_POINT;
extern const double secondsToRun;
extern const double x_strip_threshold;

//Function to normalise heading in degrees
inline double normHeading(double heading) {
	if (heading > 360.0) { return heading - 360.0; }
	else if (heading < 0.0) { return heading + 360.0; }
	else { return heading; }
}

//Structure for storing data about blackline
struct RectData {
	bool detected;
	double heading, angle;
	cv::Point2f center;
	cv::RotatedRect maxRect;
};


//Abstract base class for states
class State : public boost::enable_shared_from_this<State> {
public:
	State() {}
	virtual ~State() {}
	virtual boost::shared_ptr<State> gotFrame(cv::Mat, RectData rectData) = 0;
};

//Main class for linefollower node
class LineFollower
{
public:
	LineFollower();
	~LineFollower();

	int loopRateHz;

	void start();
	void stop();

	void publishMovement(bbauv_msgs::ControllerGoal goal);

	void compassCallback(const bbauv_msgs::compass_data& msg);
	void bottomCamCallback(const sensor_msgs::ImageConstPtr& msg);

	//Fill rectData structure with necessary data
	void prepareBlackLineParams(cv::Mat inImage);
private:
	bool enabled;

	ros::NodeHandle nh;
	ros::NodeHandle private_nh;
	//Subscribers to respective topic
	image_transport::Subscriber imageSub;
	ros::Subscriber compassSub;
	image_transport::ImageTransport it;

        //Publisher
        ros::Publisher outputPub;

	//Actionlib
	actionlib::SimpleActionClient <bbauv_msgs::ControllerAction> ac;

	//Center detection parameter
	double thVal;
	double areaThresh;
	RectData rectData;
	cv::Size screen;

	//States
	boost::shared_ptr<State> state;
};

//-------------------

class LookForLineState : public State {
private:
	LineFollower* follower;
public:
	LookForLineState(LineFollower* fl);
	boost::shared_ptr<State> gotFrame (cv::Mat, RectData rectData);
};

//--------------------

class TemporaryState : public State {
private:
	LineFollower* follower;
	double transitionTime;
	boost::shared_ptr<State> nextState;
	double speed;
public:
	TemporaryState(LineFollower* fl,
				   double secondsToReverse,
				   boost::shared_ptr<State> nextState,
				   double speed);
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData rectData);
};

//---------------------

class DiveState : public State {
private:
	LineFollower* follower;
	double transitionTime;
	boost::shared_ptr<State> nextState;
public:
	DiveState (LineFollower* fl, double secondsToDive, boost::shared_ptr<State> nextState);
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
};

//---------------------

class SurfaceState : public State {
private:
	LineFollower* follower;
public:
	SurfaceState(LineFollower* fl, double heading);
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
};

//---------------------

class StraightLineState : public State {
private:
	LineFollower* follower;
public:
	StraightLineState(LineFollower* fl);
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
};


#endif /* LINEFOLLOWINGSTATES_H_ */

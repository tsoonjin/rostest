/*
 * flarestates.h
 *
 *  Created on: 24 Jan, 2014
 *      Author: ndt
 */

#ifndef FLARESTATES_H_
#define FLARESTATES_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include "bbauv_msgs/controller.h"
#include "bbauv_msgs/compass_data.h"
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
extern const int DEPTH_POINT;
extern ros::Publisher movementPub;

//Structure for bounding box
struct RectData{
	bool detected;
	double heading, angle;
	cv::Point2f center;
	cv::RotatedRect maxRect;
};

//-----------------

//Abstract base class for states
class State : public boost::enable_shared_from_this<State>{
public:
	State() {};
	virtual ~State() {};
	virtual boost::shared_ptr<State> gotFrame(cv::Mat, RectData rectData) = 0;
};

//--------------

class FlareDetection
{
public:
	FlareDetection();
	~FlareDetection();

	int loopRateHz;

	void start();
	void stop();

	void compassCallback(const bbauv_msgs::compass_data& msg);
	double normHeading(double heading);
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void publishMovement(bbauv_msgs::ControllerGoal goal);

	//Fill rectData structure with necessary data
	void prepareFlareParams(cv::Mat inImage);

	//Threshold parameters
	int lowerH, higherH, lowerS, higherS, lowerV, higherV;
private:
	bool enabled;

	ros::NodeHandle nh;
	ros::NodeHandle private_nh;

	image_transport::Subscriber imageSub;
	ros::Subscriber compassSub;
	image_transport::ImageTransport it;

	//States
	boost::shared_ptr<State> state;

	//Action library
	actionlib::SimpleActionClient <bbauv_msgs::ControllerAction> ac;

	//Center detection parameter
	double areaThresh;
	RectData rectData;
	cv::Size screen;

};

//--------------------------

//Look for flare state
class LookForFlareState : public State{
public:
	LookForFlareState(FlareDetection *fd);
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
private:
	FlareDetection* detector;
};

//Lost flare state
class LostFlareState : public State{
public:
	LostFlareState();
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
};

//Surface
class SurfaceState : public State {
public:
	SurfaceState(double heading);
	boost::shared_ptr<State> gotFrame(cv::Mat, RectData);
};

#endif /* FLARESTATES_H_ */

/* 
	red_bucket_detection.cpp
	Date created: Jan 2014
	Author: Jason Poh
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>

#include <bbauv_msgs/compass_data.h>
#include <bbauv_msgs/controller.h>
#include "bucketstates.h"

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

const int DEPTH_POINT = 1.1;
ros::Publisher movementPub;

class BucketDetection
{
public:
	BucketDetection();
	~BucketDetection();

	int loopRateHz;

	void start();
	void stop();

	void compassCallback(const bbauv_msgs::compass_data& msg);
	double normHeading(double heading);
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);

	//Fill rectData structure with necessary data
	void prepareBucketParams(Mat inImage);
private:
	bool enabled;

	ros::NodeHandle nh;
	ros::NodeHandle private_nh;

	image_transport::Subscriber imageSub;
	ros::Subscriber compassSub;
	image_transport::ImageTransport it;

	//States
	boost::shared_ptr<State> state;
};

BucketDetection::BucketDetection(): it(nh) {
	enabled = false;

	private_nh.param<int>("loopHz", loopRateHz, 20);
	string imageTopic; private_nh.param<std::string>("image", imageTopic, "/bottomcam/camera/image_rect_color");
	string compassTopic; private_nh.param<std::string>("compass", compassTopic, "/compass");

 	imageSub = it.subscribe(imageTopic, 1, &BucketDetection::imageCallback, this);
    compassSub = nh.subscribe(compassTopic, 1, &BucketDetection::compassCallback, this);
	movementPub = nh.advertise<bbauv_msgs::controller>("/movement", 1);
}

BucketDetection::~BucketDetection() {

}

void BucketDetection::start() {
	enabled = true;
}

void BucketDetection::stop() {
	enabled = false;
}

void BucketDetection::compassCallback(const bbauv_msgs::compass_data& msg) {

}

void BucketDetection::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
 	cv_bridge::CvImagePtr cv_ptr;	
	try {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
	}catch(cv_bridge::Exception& e){
		cv_bridge::CvImagePtr cv_ptr;
	}

	prepareBucketParams(cv_ptr->image);
}

void BucketDetection::prepareBucketParams(Mat image) {
	//Do image processing here

	//Call state

}

int main(int argc, char **argv)
{
	/*
		Initalization
	*/
	ros::init(argc, argv, "red_bucket_detection");

	BucketDetection bucketDetector;
	bucketDetector.start();
	ROS_INFO("Initialised Bucket Detection...");

	ros::Rate loop_rate(bucketDetector.loopRateHz);
	while (ros::ok()) {
		ros::spinOnce();
		loop_rate.sleep();
	}

  	ros::spin();
	return 0;
}

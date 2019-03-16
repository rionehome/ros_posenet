#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <memory>
#include "json11.hpp"
#include "ros_posenet/Keypoint.h"
#include "ros_posenet/Poses.h"
#include "ros_posenet/Pose.h"

using namespace std;

int main(int argc, char **argv) {

	ros::init(argc, argv, "sample");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/js_poses", 1000, callback);
	ros::Subscriber left_sub = n.subscribe("/openni2/depth", 1, getImage_depth);

	ros::spin();

	return 0;
}
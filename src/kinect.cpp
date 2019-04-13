#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include "json11.hpp"
#include "ros_posenet/Keypoint.h"
#include "ros_posenet/Poses.h"
#include "ros_posenet/Pose.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>


#define KINECT_WIDTH 640
#define KINECT_HEIGHT 480

using namespace std;
ros::Publisher posenet_image;
ros::Publisher posenet_poses;
cv::Mat depth_data = cv::Mat::zeros(480, 640, CV_64F);

void view_depth_image(cv::Mat depth) {
	cv::Mat drawable;
	cv::Mat raw_input = cv::Mat::zeros(480, 640, CV_64F);
	for (int x = 0; x < 640; ++x) {
		for (int y = 0; y < 480; ++y) {
			raw_input.at<double>(y, x) = (depth.at<cv::Vec3b>(y, x)[1] * 100 + depth.at<cv::Vec3b>(y, x)[2]) * 255 / 4000.0;
		}
	}
	raw_input.convertTo(drawable, CV_8UC1);
	cv::imshow("window", drawable);
	cv::waitKey(1);
}

void depth_callback(const sensor_msgs::Image::ConstPtr& msg) {
	cv::Mat depth_image;
	try {
		// ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
		depth_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB16)->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	for (int x = 0; x < 640; ++x) {
		for (int y = 0; y < 480; ++y) {
			depth_data.at<double>(y, x) = (depth_image.at<cv::Vec3b>(y, x)[1] * 100 + depth_image.at<cv::Vec3b>(y, x)[2]) * 255 / 4000.0;
		}
	}
}

void color_callback(const sensor_msgs::Image::ConstPtr& msg) {
	posenet_image.publish(msg);
}

void poses_callback(const std_msgs::String::ConstPtr& msg) {

	string err;
	auto json = json11::Json::parse(msg->data, err);
	ros_posenet::Poses poses;
	if (json["poses"].array_items().size() != 0) {
		for (auto &p : json["poses"].array_items()) {

			ros_posenet::Pose pose;
			for (auto &k : p["keypoints"].array_items()) {
				if (std::stod(k["score"].dump()) > 0.5) {

					ros_posenet::Keypoint key;

					key.position.x = std::stod(k["position"]["x"].dump());
					key.position.y = std::stod(k["position"]["y"].dump());
					//key.position.z = depth_vector[KINECT_HEIGHT * key.position.x + key.position.y];
					//key.position.z = depth_vector[KINECT_WIDTH * key.position.y + key.position.x];
					//printf("%d\n", (int)depth_data.at<double>(key.position.y, key.position.x) );
					key.position.z = (int)depth_data.at<double>(key.position.y, key.position.x) / 1000.0;
					//key.position.z = 0;
					key.score = std::stod(k["score"].dump());
					key.part = k["part"].dump();
					pose.keypoints.push_back(key);
				}
			}
			poses.poses.push_back(pose);
		}
		posenet_poses.publish(poses);
	} else {
		printf("error\n");
	}
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "kinect");

	ros::NodeHandle n;

	ros::Subscriber color = n.subscribe("/ros_kinect/color", 1, color_callback);
	ros::Subscriber depth = n.subscribe("/ros_kinect/depth", 1, depth_callback);
	posenet_image = n.advertise<sensor_msgs::Image>("/posenet/input", 1);
	ros::Subscriber poses = n.subscribe("/posenet/output", 1, poses_callback);
	posenet_poses = n.advertise<ros_posenet::Poses>("/ros_posenet/poses", 1);

	ros::spin();

	return 0;
}

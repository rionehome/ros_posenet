#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <memory>
#include "json11.hpp"

using namespace std;

cv::Mat color;

void getImage_color(const sensor_msgs::Image::ConstPtr& msg) {

	cv_bridge::CvImagePtr cv_ptr;

	try {
		// ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	color = cv_ptr->image;
}

void view(const std_msgs::String::ConstPtr& msg) {

	string err;
	auto json = json11::Json::parse(msg->data, err);

	if (json["poses"].array_items().size() != 0) {
		for (auto &p : json["poses"].array_items()) {
			for (auto &k : p["keypoints"].array_items()) {
				if (std::stod(k["score"].dump()) > 0.5) {

					double x = std::stod(k["position"]["x"].dump());
					double y = std::stod(k["position"]["y"].dump());

					cv::circle(color, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1);
				}
			}
		}
		cv::imshow("Color", color );
		cv::waitKey(1);
	} else {
		printf("error\n");
	}
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "viewer");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/poses", 1000, view);
	ros::Subscriber left_sub = n.subscribe("/openni2/color", 1, getImage_color);


	ros::spin();

	return 0;
}
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <memory>
#include "json11.hpp"

using namespace std;

cv::Mat depth;

void getImage_depth(const sensor_msgs::Image::ConstPtr& msg) {

	cv_bridge::CvImagePtr cv_ptr;

	try {
		// ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット。
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	depth = cv_ptr->image;
}

void callback(const std_msgs::String::ConstPtr& msg) {

	string err;
	auto json = json11::Json::parse(msg->data, err);

	if (json["poses"].array_items().size() != 0) {
		for (auto &p : json["poses"].array_items()) {
			for (auto &k : p["keypoints"].array_items()) {
				if (std::stod(k["score"].dump()) > 0.5) {

					double x = std::stod(k["position"]["x"].dump());
					double y = std::stod(k["position"]["y"].dump());
					double score = std::stod(k["score"].dump());

					//cv::circle(depth, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), -1);
				}
			}
		}
		//cv::imshow("Color", depth );
		//cv::waitKey(1);
	} else {
		printf("error\n");
	}
}


int main(int argc, char **argv) {

	ros::init(argc, argv, "injection");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/poses", 1000, callback);
	ros::Subscriber left_sub = n.subscribe("/openni2/depth", 1, getImage_depth);


	ros::spin();

	return 0;
}
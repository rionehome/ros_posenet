#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include "json11.hpp"
#include "ros_posenet/Keypoint.h"
#include "ros_posenet/Poses.h"
#include "ros_posenet/Pose.h"

using namespace std;
ros::Publisher posenet_image;
ros::Publisher posenet_poses;

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
					key.position.z = 0;
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

	ros::Subscriber color = n.subscribe("/ros_kinect/color", 1000, color_callback);
	//ros::Subscriber depth = n.subscribe("/ros_kinect/depth", 1000, image_callback);
	posenet_image = n.advertise<sensor_msgs::Image>("/posenet/input", 1000);
	ros::Subscriber poses = n.subscribe("/posenet/output", 1, poses_callback);
	posenet_poses = n.advertise<ros_posenet::Poses>("/ros_posenet/poses", 1000);

	ros::spin();

	return 0;
}
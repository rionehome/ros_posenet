#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include "module/json11.hpp"
#include <string>
#include "ros_posenet/Keypoint.h"
#include "ros_posenet/Poses.h"
#include "ros_posenet/Pose.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include "../include/ros_posenet/realsense.h"

Realsense::Realsense(ros::NodeHandle *n)
{
    printf("Start class of 'Realsense'\n");
    this->point_cloud_sub =
        n->subscribe("/camera/depth_registered/points", 1, &Realsense::point_cloud_data_callback, this);
    this->output_sub = n->subscribe("/posenet/output", 1, &Realsense::poses_callback, this);
    this->input_pub = n->advertise<sensor_msgs::Image>("/posenet/input", 1);
    this->posenet_result_pub = n->advertise<ros_posenet::Poses>("/ros_posenet/result", 1);
    this->posenet_image_result_pub = n->advertise<ros_posenet::Poses>("/ros_posenet/image_result", 1);
}

Realsense::~Realsense()
{
    printf("Shutdown class of 'Realsense'\n");
}

void Realsense::point_cloud_data_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
{
    if (received) return;

    int height = (int) input->height;
    int width = (int) input->width;
    double z;

    color = cv::Mat::zeros(cv::Size(width, height), CV_8UC3);
    depth = cv::Mat::zeros(cv::Size(width, height), CV_64F);

    //pcl変換
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

    this->pc = *temp_cloud;

    //展開
    for (int w = 0; w < width; ++w) {
        for (int h = 0; h < height; ++h) {
            color.at<cv::Vec3b>(h, w)[0] = (int) temp_cloud->points[width * h + w].r;
            color.at<cv::Vec3b>(h, w)[1] = (int) temp_cloud->points[width * h + w].g;
            color.at<cv::Vec3b>(h, w)[2] = (int) temp_cloud->points[width * h + w].b;
            z = temp_cloud->points[width * h + w].z;
            depth.at<double>(h, w) = z;
        }
    }

    depth.convertTo(view_depth, CV_8UC1, 255.0 / 4.0);

    cv::namedWindow("depth", CV_WINDOW_NORMAL);
    cv::imshow("depth", view_depth);

    cv::waitKey(1);

    this->input_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "rgb8", color));
    this->received = true;

}

void Realsense::poses_callback(const std_msgs::String::ConstPtr &msg)
{
    std::string err;
    auto json = json11::Json::parse(msg->data, err);
    ros_posenet::Poses poses;
    ros_posenet::Poses image_poses;
    int image_x, image_y;
    cv::Point3d real_position;

    if (json["poses"].array_items().empty())
        printf("error\n");
    else {
        for (auto &p : json["poses"].array_items()) {
            ros_posenet::Pose pose;
            ros_posenet::Pose image_pose;
            for (auto &k : p["keypoints"].array_items()) {
                if (std::stod(k["score"].dump()) > 0.5) {
                    ros_posenet::Keypoint key;
                    ros_posenet::Keypoint image_key;
                    image_x = (int) std::stod(k["position"]["x"].dump());
                    image_y = (int) std::stod(k["position"]["y"].dump());
                    real_position = get_real_point_data(&pc, color.cols, cv::Point(image_x, image_y));

                    key.position.x = real_position.x;
                    key.position.y = real_position.y;
                    key.position.z = real_position.z;
                    key.score = std::stod(k["score"].dump());
                    key.part = k["part"].dump();
                    key.part.erase(remove(key.part.begin(), key.part.end(), '"'), key.part.end());

                    image_key.position.x = image_x;
                    image_key.position.y = image_y;
                    image_key.position.z = real_position.z;
                    image_key.score = std::stod(k["score"].dump());
                    image_key.part = k["part"].dump();
                    image_key.part
                        .erase(remove(image_key.part.begin(), image_key.part.end(), '"'), image_key.part.end());

                    /*
                    cv::Point3d result;
                    int i = 1;
                    while (search_around(i++, cv::Point((int) image_x, (int) image_y), color.cols, &result));
                    key.position.z = result.z;
                    */

                    pose.keypoints.push_back(key);
                    image_pose.keypoints.push_back(image_key);
                }
            }
            poses.poses.push_back(pose);
            image_poses.poses.push_back(image_pose);
        }
        this->posenet_result_pub.publish(poses);
        this->posenet_image_result_pub.publish(image_poses);
    }
    this->received = false;
}

cv::Point3d Realsense::get_real_point_data(pcl::PointCloud<pcl::PointXYZRGB> *data, int width, const cv::Point &image)
{
    double z = depth.at<double>(image.y, image.x);
    if (!(z >= 0.4 && z <= 4.0)) return cv::Point3d(0.0, 0.0, 0.0);
    auto point = data->points[width * image.y + image.x];
    return cv::Point3d(point.x, point.y, point.z);
}

bool Realsense::search_around(int area, const cv::Point &image_center, int width, cv::Point3d *result)
{
    //areaは奇数に限る
    if (area % 2 == 0) return true;
    if (area == 1) {
        *result = get_real_point_data(&pc, width, image_center);
        return result->z == 0.0;
    }
    cv::Point min, max;
    cv::Point3d sum, tmp;
    double count = 0;

    min.x = image_center.x - (int) (area / 2);
    min.y = image_center.y - (int) (area / 2);
    max.x = image_center.x + (int) (area / 2);
    max.y = image_center.y + (int) (area / 2);

    for (int y = min.y; y < max.y; ++y) {
        for (int x = min.x; x < max.x; ++x) {
            tmp = get_real_point_data(&pc, width, cv::Point(x, y));
            if (tmp.z == 0.0) continue;
            sum.x += tmp.x;
            sum.y += tmp.y;
            sum.z += tmp.z;
            count++;
        }
    }

    if (count == 0) return true;

    result->x = sum.x / count;
    result->y = sum.y / count;
    result->z = sum.z / count;

    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "realsense");
    ros::NodeHandle n;
    Realsense realsense(&n);
    ros::spin();
    return 0;
}
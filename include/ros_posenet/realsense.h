//
// Created by migly on 19/08/04.
//

#ifndef REALSENSE_H
#define REALSENSE_H

class Realsense
{
public:
    explicit Realsense(ros::NodeHandle *n);
    ~Realsense();

private:
    bool received = false;
    cv::Mat color;
    cv::Mat depth;
    cv::Mat view_depth;
    pcl::PointCloud<pcl::PointXYZRGB> pc;

    ros::Subscriber output_sub;
    ros::Subscriber point_cloud_sub;
    ros::Publisher input_pub;
    ros::Publisher posenet_result_pub;

    void poses_callback(const std_msgs::String_<std::allocator<void>>::ConstPtr &msg);
    void point_cloud_data_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input);
    cv::Point3d get_real_point_data(pcl::PointCloud<pcl::PointXYZRGB> *data, int width, const cv::Point &image);
    bool search_around(int area, const cv::Point &image_center, int width, cv::Point3d *result);
};

#endif //REALSENSE_H

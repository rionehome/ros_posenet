//
// Created by migly on 19/08/04.
//

#ifndef KINECT_H
#define KINECT_H

class Kinect
{
public:
    explicit Kinect(ros::NodeHandle *n);
    ~Kinect();

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
};


#endif //KINECT_H

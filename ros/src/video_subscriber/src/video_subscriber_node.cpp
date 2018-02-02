#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Received image with size: %i x %i", msg->width, msg->height);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle n;

    std::cout<<"Subscribing starts!"<<std::endl;
    ros::Subscriber sub = n.subscribe("/sd5_1/camera_sd5_left/image_raw", 2, imageCallback);

    ros::spin();
    return 0;
}

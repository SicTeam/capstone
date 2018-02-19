#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "../../../../tracking/drone.h"

mavros_msgs::State current_state;
Track drone_track;

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// Gets called when a left image frame comes in
void image_callback_left(const sensor_msgs::ImageConstPtr& msg) {
    ROS_INFO("Received left image with size: %i x %i", msg->width, msg->height);
}

// Gets called when a right image frame comes in
void image_callback_right(const sensor_msgs::ImageConstPtr& msg) {
    //ROS_INFO("Received right image with size: %i x %i", msg->width, msg->height);
}

int main(int argc, char **argv) {

    // ROS required initialization
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    // Create subscribers and publishers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/sd5_1/mavros/state", 10, state_cb);
    ros::Subscriber left_sub = nh.subscribe("/sd5_1/camera_sd5_left/image_raw",
            2, image_callback_left);
    ros::Subscriber right_sub = nh.subscribe("/sd5_1/camera_sd5_right/image_raw",
            2, image_callback_right);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/sd5_1/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("/sd5_1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/sd5_1/mavros/set_mode");

    // The setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // Wait for FCU connection
    while(ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // Create a position 2 meters upwards
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    // Send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // OFFBOARD mavlink command
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // Arming command
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()) {
        // Send OFFBOARD command every 5s until success
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            // Send arming command every 5s until success
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // Send positioning command
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


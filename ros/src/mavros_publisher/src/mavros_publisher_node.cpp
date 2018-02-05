#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

mavros_msgs::State current_state_sd5;
mavros_msgs::State current_state_uav2;

void state_sd5_callback(const mavros_msgs::State::ConstPtr& msg) {
    current_state_sd5 = *msg;
}

void state_uav2_callback(const mavros_msgs::State::ConstPtr& msg) {
    current_state_uav2 = *msg;
}

void image_callback_left(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Received left image with size: %i x %i", msg->width, msg->height);
}

void image_callback_right(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("Received right image with size: %i x %i", msg->width, msg->height);
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    // Recieve frames from left and right cameras
    ros::Subscriber left_sub = nh.subscribe("/sd5_1/camera_sd5_left/image_raw",
            2, image_callback_left);
    ros::Subscriber right_sub = nh.subscribe("/sd5_1/camera_sd5_right/image_raw",
            2, image_callback_right);
    
    // SicDrone SD5
    ros::Subscriber state_sub_sd5 = nh.subscribe<mavros_msgs::State>
            ("/sd5_1/mavros/state", 10, state_sd5_callback);
    ros::Publisher local_pos_pub_sd5 = nh.advertise<geometry_msgs::PoseStamped>
            ("/sd5_1/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client_sd5 = nh.serviceClient<mavros_msgs::CommandBool>
            ("/sd5_1/mavros/cmd/arming");
    ros::ServiceClient set_mode_client_sd5 = nh.serviceClient<mavros_msgs::SetMode>
            ("/sd5_1/mavros/set_mode");
    
    // UAV 2
    ros::Subscriber state_sub_uav2 = nh.subscribe<mavros_msgs::State>
            ("/uav2/mavros/state", 10, state_uav2_callback);
    ros::Publisher local_pos_pub_uav2 = nh.advertise<geometry_msgs::PoseStamped>
            ("/uav2/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client_uav2 = nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav2/mavros/cmd/arming");
    ros::ServiceClient set_mode_client_uav2 = nh.serviceClient<mavros_msgs::SetMode>
            ("/uav2/mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state_sd5.connected  && !current_state_uav2.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pos_hold;
    pos_hold.pose.position.x = 0;
    pos_hold.pose.position.y = 0;
    pos_hold.pose.position.z = 2;
    
    geometry_msgs::PoseStamped pos_left;
    pos_left.pose.position.x = -10;
    pos_left.pose.position.y = 0;
    pos_left.pose.position.z = 2;
    
    geometry_msgs::PoseStamped pos_right;
    pos_right.pose.position.x = 10;
    pos_right.pose.position.y = 0;
    pos_right.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub_sd5.publish(pos_hold);
        local_pos_pub_uav2.publish(pos_hold);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode_sd5;
    offb_set_mode_sd5.request.custom_mode = "OFFBOARD";
    mavros_msgs::SetMode offb_set_mode_uav2;
    offb_set_mode_uav2.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd_sd5;
    arm_cmd_sd5.request.value = true;
    mavros_msgs::CommandBool arm_cmd_uav2;
    arm_cmd_uav2.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state_sd5.mode != "OFFBOARD" && current_state_uav2.mode != "OFFBOARD" 
                && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client_sd5.call(offb_set_mode_sd5) &&
                offb_set_mode_sd5.response.mode_sent){
                ROS_INFO("Offboard enabled SD5");
            }
            if( set_mode_client_uav2.call(offb_set_mode_uav2) &&
                offb_set_mode_uav2.response.mode_sent){
                ROS_INFO("Offboard enabled UAV2");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state_sd5.armed && !current_state_uav2.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client_sd5.call(arm_cmd_sd5) &&
                    arm_cmd_sd5.response.success){
                    ROS_INFO("Vehicle armed SD5");
                }
                if( arming_client_uav2.call(arm_cmd_uav2) &&
                    arm_cmd_uav2.response.success){
                    ROS_INFO("Vehicle armed UAV2");
                }
                last_request = ros::Time::now();
            }
        }

        ROS_INFO("%s", ros::Time::now());
        local_pos_pub_sd5.publish(pos_hold);
        local_pos_pub_uav2.publish(pos_hold);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}


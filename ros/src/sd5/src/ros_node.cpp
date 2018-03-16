/*

The MIT License

Copyright (c) 2018 SicTeam - (Portland State University)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include <ctime>
#include <cstdlib>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "../../../../tracking/drone.h"

// Write video out to files
const bool EXPORT_VID = false;

struct frame_data {
    cv::Mat frame;
    std::vector<cv::Rect> rects;
} typedef frame_data;

// Store camera frames and bounding boxes
std::deque<frame_data> left_images;
std::deque<frame_data> right_images;

// Current FCU state
mavros_msgs::State current_state;

// Position
geometry_msgs::PoseStamped current_pos;
geometry_msgs::PoseStamped goto_pos;

// Last pursue message sent
//ros::Time last_pursue;

// If takeoff executed
bool airborne = false;

// Flag when armed
bool armed = false;

// Drone detecting/tracking object
Track * drone_track;

// Drone detected in last frame flag
bool detect_left = false;

// Tracking object for tracking function
cv::Ptr<cv::Tracker> track_left;

cv::VideoWriter vwLeft("left.avi",
        cv::VideoWriter::fourcc('D', 'I', 'V', 'X'),
        15.0,
        cv::Size(800, 800),
        true);
cv::VideoWriter vwRight("right.avi",
        cv::VideoWriter::fourcc('D', 'I', 'V', 'X'),
        15.0,
        cv::Size(800, 800),
        true);

// DEBUGGING
void print_list() {
    std::cout << "FRAME LIST" << std::endl;
    for (int i = 0; i < left_images.size(); i++) {
        std::cout << "mat " << i << ": " << left_images[i].frame.size << std::endl;
        std::cout << "rects size: " << left_images[i].rects.size() << std::endl;
    }
    std::cout << "END FRAME LIST" << std::endl;
}

// Get current mavros state
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_pos = *msg;
}


// Gets called when a left image frame comes in
void image_callback_left(const sensor_msgs::ImageConstPtr& msg) {
    static int left_count = 0;

    if (left_count == 2) {
        //ROS_INFO("Received left image");

        // Extract cv::Mat from message
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat left_image = cv_ptr->image;

        if (left_images.size() == 20) {
            left_images.pop_front();
        }

        std::vector<cv::Rect> rects;

        // Detect and draw bounding boxes on frame
	    //if(!detectLeft){
        //    ROS_INFO("Conducting Left Detection");
        detect_left = drone_track->detect(rects, left_image);
        //}
	    //else{
        //    ROS_INFO("Conducting Left Tracking");
        //	drone_track->detect(dronesLeft, left_image);
		//    detectLeft = drone_track->track(left_image, drones_left[0], track_left);
        //}

        frame_data data;
        data.frame = left_image;
        data.rects = rects;
        left_images.push_back(data);

        //print_list();

        if (EXPORT_VID) {
            vwLeft << left_image;
        }

        left_count = 0;
    } else {
        left_count++;
    }
}

// Gets called when a right image frame comes in
void image_callback_right(const sensor_msgs::ImageConstPtr& msg) {
    static int right_count = 0;

    if(right_count == 2) {
        //ROS_INFO("Received right image");

        // Extract cv::Mat from message
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat right_image = cv_ptr->image;

        if (right_images.size() == 20) {
            right_images.pop_front();
        }

        // Detect and draw bounding boxes on frame
        std::vector<cv::Rect> rects;
        drone_track->detect(rects, right_image);
        
        frame_data data;
        data.frame = right_image;
        data.rects = rects;
        right_images.push_back(data);

        if (EXPORT_VID) {
            vwRight << right_image;
        }
        right_count = 0;
    } else {
        right_count++;
    }
}

// Gets called when a left image frame comes in
void image_callback_rear(const sensor_msgs::ImageConstPtr& msg) {

    // Extract cv::Mat from message
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat rear_image = cv_ptr->image;

    std::vector<cv::Rect> rects;
    drone_track->detect(rects, rear_image);
}

bool should_pursue(ros::Time& last_pursue_time) {
    int size = left_images.size();

    // We need at least 5 data points
    if (size < 5) {
        return false;
    }

    // airborne is set when armed and 10 seconds has passed
    if (!airborne) {
        return false;
    }

    // Don't send command if we already have in the last 5 seconds
    if (ros::Time::now() - last_pursue_time < ros::Duration(5.0)) {
        return false;
    }

    // Last 5 camera frames need to have detected something
    for (int i = size; i > size - 5; i--) {
        if (left_images[i].rects.size() == 0) {
            return false;
        }
    }

    return true;
}

geometry_msgs::Point compute_next_pos(float x, float y, float z) {
    geometry_msgs::Point new_pos;

    if (std::rand() % 2 == 0) {
        y = -y;
    }

    // Append current position to new coordinates
    x += current_pos.pose.position.x;
    y += current_pos.pose.position.y;

    new_pos.x = x;
    new_pos.y = y;
    new_pos.z = z;

    return new_pos;
}

int main(int argc, char **argv) {

    // ROS required initialization
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    // Init rand
    std::srand(std::time(NULL));

    // Get $HOME
    std::string home(getenv("HOME"));
    std::string cascade_loc = home + "/cascade.xml";

    drone_track = new Track(cascade_loc);

    // Create subscribers and publishers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/sd5_1/mavros/state", 10, state_cb);
    ros::Subscriber left_sub = nh.subscribe("/sd5_1/camera_sd5_left/image_raw",
            2, image_callback_left);
    ros::Subscriber right_sub = nh.subscribe("/sd5_1/camera_sd5_right/image_raw",
            2, image_callback_right);
    ros::Subscriber rear_sub = nh.subscribe("/sd5_1/camera_sd5_rear/image_raw",
            2, image_callback_rear);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/sd5_1/mavros/local_position/pose", 2, pose_callback);
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

    // Give default values to current pos
    current_pos.pose.position.x = 0;
    current_pos.pose.position.y = 0;
    current_pos.pose.position.z = 0;

    // Create a an initial (takeoff) position 5 meters up
    geometry_msgs::PoseStamped takeoff_pos;
    takeoff_pos.pose.position.x = 0;
    takeoff_pos.pose.position.y = 0;
    takeoff_pos.pose.position.z = 5;

    goto_pos.pose.position.x = 0;
    goto_pos.pose.position.y = 0;
    goto_pos.pose.position.z = 5;

    // Send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(takeoff_pos);
        ros::spinOnce();
        rate.sleep();
    }

    // OFFBOARD mavlink command
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // Arming command
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // Keep track of requests to prevent command overload
    ros::Time last_request_time = ros::Time::now();
    // Keep track of pursue commands to prevent command overload
    ros::Time last_pursue_time = ros::Time::now();
    // Keep track of arm time to signal when airborne
    ros::Time arm_time;

    while(ros::ok()) {
        // Send OFFBOARD command every 5s until success
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request_time > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request_time = ros::Time::now();
        } else {
            // Send arming command every 5s until success
            if (!current_state.armed &&
                (ros::Time::now() - last_request_time > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    arm_time = ros::Time::now();
                    armed = true;
                    ROS_INFO("Vehicle armed");
                }
                last_request_time = ros::Time::now();
            }
        }


        // If 10 seconds has passed since arming, takeoff command should have already executed and completed
        if (armed && (ros::Time::now() - arm_time > ros::Duration(10.0))) {
            airborne = true;
        }

        if (should_pursue(last_pursue_time)) {
            ROS_INFO("Pursuing");
            // Compute next position as 3m forward, +/- 3m sideways, and same altitude
            geometry_msgs::Point next_point = compute_next_pos(3.0, 3.0, current_pos.pose.position.z);
            goto_pos.pose.position = next_point;

            // Send movement command
            local_pos_pub.publish(goto_pos);

            last_pursue_time = ros::Time::now();
        } else {
            // If we're not pursuing, just feed the drone it's current position to keep it happy
            local_pos_pub.publish(goto_pos);
//            std::cout << "Current pos:" << std::endl;
//            std::cout << current_pos.pose.position.x << " " << current_pos.pose.position.y << " " << current_pos.pose.position.z << std::endl;
        }
        
	    ros::spinOnce();
        rate.sleep();
    }

    delete drone_track;

    return 0;
}


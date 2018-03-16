//This file holds the class interface for the class that contains tracking
//features


#include <iostream>
#include <cmath>//abs
#include <geometry_msgs/PoseWithCovariance.h>//post messages
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

//#include <stdio.h>

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()



//Task:
class Track
{
    public:
        Track();
        Track(std::string file_name);
        int detect_image();
    	int detect_image(std::string image);
        int detect(std::vector<cv::Rect> & drones, cv::Mat frame);
        int kcf(char * vid);
        int kcf(char * vid1, char * vid2);
        int kcf();
        void display();
	int track(cv::Mat & frame, cv::Rect & drone, cv::Ptr<cv::Tracker> & tracker);
	int track_tester(char * vid);

   int depth(cv::Point left, cv::Point right);
   void projectedXY(cv::Point center);
    private:
        void createTracker(cv::Ptr<cv::Tracker>& tracker, const std::string& trackerType);
        cv::Rect target;
        cv::Point image_center;
        cv::Point target_point;
	    cv::Point left_target_point;
	    cv::Point right_target_point;

        cv::CascadeClassifier cascade;
        std::string cascade_name;
        int min_neighbors;        
	    bool cam2_detect;

	//to store old frame
	cv:: Mat mat_store[2];
	cv:: Point target_store;
};

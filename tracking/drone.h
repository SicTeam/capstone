//This file holds the class interface for the class that contains tracking
//features


#include <iostream>
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
        int detect(std::vector<cv::Rect> & drones, cv::Mat frame);
        int kcf(char * vid);
        int kcf();
        void display();

    private:
        cv::Rect target;
        cv::Point target_point;
        cv::CascadeClassifier cascade;
        std::string cascade_name;
        int min_neighbors;        
};

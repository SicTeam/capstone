//Copyright <2018> <COPYRIGHT HOLDER>

//Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

//The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

//This file will hold the inner workings of any tracking feature. The content we used in this file is taken from OpenCV library.
//This file holds the class interface for the class that contains tracking features

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

//Task:
class Track
{
    public:
        Track();  	//constructor	
        Track(std::string file_name); //constructor
    	int detect_image(std::string image); //detect in a single image
        int detect(std::vector<cv::Rect> & drones, cv::Mat frame); //detect in a video
	int track(cv::Mat & frame, cv::Rect & drone, cv::Ptr<cv::Tracker> & tracker); //tracking functin
	int track_tester(char * vid); //test function for track

	//need these ? 
        int kcf(char * vid);
        int kcf(char * vid1, char * vid2);
        int kcf();
    private:
        void createTracker(cv::Ptr<cv::Tracker>& tracker, const std::string& trackerType);//bound the tracker object with tracker type
        
        cv::CascadeClassifier cascade; //holder for training data
        std::string cascade_name; //name of training data
        int min_neighbors; //used for detection algorithm

	//need these ? 
	cv::Rect target;
        cv::Point target_point;
	cv::Point left_target_point;
	cv::Point right_target_point;
	bool cam2_detect;
	//to store old frame
	cv:: Mat mat_store[2];
	cv:: Point target_store;
};

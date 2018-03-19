/*

The MIT License

Copyright (c) 2018 SicTeam - (Portland State University)

SicTeam is: Israel Bond, Brandon Craig, Cody Herberholz, Khuong Nguyen,
            Dakota Sanchez, Samuel Strba, Elijah Whitham-Powell

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
#include "drone.h"

//Track class constructor
Track::Track()
{
    //cascade_name = "cascade.xml"; // name of training data in Real Life
    cascade_name = "sim_cascade.xml"; //name of training data in Simulation
    min_neighbors = 15; //the higher this number the more strict detection is
    if(!cascade.load(cascade_name))
    {
        std::cout << "FAIL" << std::endl;
    }
}

//Track class constructor but let user to choose data training file
//use for testing purpose if needed
Track::Track(std::string file_name)
{
    min_neighbors = 3;//the higher this number the more strict detection is
    cascade_name = file_name;
    if(!cascade.load(cascade_name))
    {
        std::cout << "FAIL" << std::endl;
    }
}

//Task: Detects drone from test images
//Input: any image with or without the drone
//Output: a bounding box on the drone position
int Track::detect_image(std::string image)
{
    std::vector<cv::Rect> drones; //list of drones
    cv::Mat frame; // hold the image

    frame = cv::imread( image, cv::IMREAD_COLOR);//read image

    //return error if can't find drone
    if(!detect(drones, frame))
    {
        std::cout << "No object Detected" << std::endl;
        return 0;
    }
    //display info
    cv::namedWindow("Drone Detect", cv::WINDOW_AUTOSIZE);
    cv::imshow("Drone Detect", frame);
    cv::waitKey(0);

    return 0;
}

//Task:   Detects drones from video
//Input:  Takes in a vector of drones of type Rect
//Output: Outputs how many drones were detected, zero means fail
int Track::detect(std::vector<cv::Rect> & drones, cv::Mat frame)
{
    cv::Mat frame_gray;//hold the image

    //data for detection
    cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
    cv::equalizeHist(frame_gray, frame_gray);

    //detect drone
    cascade.detectMultiScale(frame_gray, drones, 1.1, min_neighbors, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

    //return 0 when no objects detected
    if(drones.size() == 0)
    {
        putText(frame, "Can't find the object", cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,225),2);
        return 0; 
    }

    int size = drones.size();
    //std::cout << "Objects Detected: " << size << std::endl;

    cv::Point center(drones[0].x + drones[0].width/2, drones[0].y + drones[0].height/2);
    
    //set class target point to pass to pursuit
    //target = center;

    //XXX determine which element of drones is actually the drone
//    for(size_t i = 0; i < size; ++i)
//    {
//        rectangle(frame, drones[i], cv::Scalar(225,0,0),2,8);
//    }
    //show the drone that will be tracked
    rectangle(frame, drones[0], cv::Scalar(225,0,0),2,8);
    circle(frame, center, 1, cv::Scalar(0,0,225), 2, 1, 0);

    return 1;
}

//Task:   bound a tracker object with a tracker type
//Input:  a tracker object, string name of tracker type
//Output: bounded tracker object with the inputed type
void Track::createTracker(cv::Ptr<cv::Tracker>& tracker, const std::string& trackerType) 
{
   
    #if (CV_MINOR_VERSION < 3)
        tracker = cv::Tracker::create(trackerType);
    #else
        if (trackerType == "BOOSTING")
            tracker = cv::TrackerBoosting::create();
        if (trackerType == "MIL")
            tracker = cv::TrackerMIL::create();
        if (trackerType == "KCF")
            tracker = cv::TrackerKCF::create();
        if (trackerType == "TLD")
            tracker = cv::TrackerTLD::create();
        if (trackerType == "MEDIANFLOW")
            tracker = cv::TrackerMedianFlow::create();
    #endif
}

//Task:   track a detected drone
//Input:  a frame hold images, list of drone, a tracker object
//Output: keep updating the location of detected drone
int Track::track(cv::Mat & frame, cv::Rect & drone, cv::Ptr<cv::Tracker> & tracker)
{
    //define and assigned tracker type
    std::string trackerTypes[5] = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW"};
    std::string trackerType = trackerTypes[4];
	 
    //tracker init
    cv::Rect2d bbox = drone;
    bool trackFail = false;
    bool cond = true; // to signal if the left camera still running
    bool ok = false;

    //bound tracker
    if(!tracker)
    {
    	createTracker(tracker, trackerType);
    }
 
    //draw a bounding box
    rectangle(frame, bbox, cv::Scalar(225, 0, 0), 2, 1);
    //XXX why we init every single time ? 
    //init tracker to frame and bounding box 
    tracker->init(frame, bbox);
    //update
    ok = tracker->update(frame, bbox);
   
    //if the target move near the box, don't draw the bounding box 
    if(bbox.x < 50 || bbox.x > (frame.rows - 50) || bbox.y < 50 || bbox.y > (frame.cols - 50))
		ok = false;
    if(ok)
    {
        //Tracking success: draw tracked object
        rectangle(frame, bbox, cv::Scalar(225, 0, 0), 2, 1);
	    return 1;
    }
    
    return 0;
} 

//Task: test for the trackers
//Input: a video live stream or recorded video, draw object to be tracked
//Output: the tracker moving the bounding box along with target movement
int Track::track_tester()
{
    //define and assing tracker type
    std::string trackerTypes[5] = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW"};
    std::string trackerType = trackerTypes[2];

    std::vector<cv::Rect> drones;// list of drones
    cv::Ptr<cv::Tracker> tracker;//create a tracker object
    cv::Mat frame; //holds video frame
    cv::Rect2d bbox;//create bounding box
    
    createTracker(tracker, trackerType);//bound tracker

    //choose which type of video to show
    	cv::VideoCapture video(0);

    //Exit if video is not opened
    if(!video.isOpened())
    {
        std::cout << "Could not read video file" << std::endl;
        return 1;
    }
    
    //take first frame
    video >> frame;

    //draw a region of interest to track
    bbox = selectROI(frame,false);
 
    //draw an actual box on screen
    rectangle(frame, bbox, cv::Scalar(225, 0, 0), 2, 1);
    //init tracker
    tracker->init(frame, bbox);

    while(video.read(frame)) 
    {
        //Update tracking result
        bool ok = tracker->update(frame, bbox);

        if(ok)
        {
            //Tracking success: draw tracked object
            rectangle(frame, bbox, cv::Scalar(225, 0, 0), 2, 1);
        }

        else
        {
	    //show user tracking failure
            cv::putText(frame, "Tracking Failure Detected", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,225),2);
        }

        // Display frame.
        cv::imshow("Tracking", frame);

        // Exit if ESC pressed.
        if(cv::waitKey(1) == 27)
        {
            break;
        }
    }
    return 0;
}


int Track::track_tester(char * vid) //test function for track
{
    //define and assing tracker type
    std::string trackerTypes[5] = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW"};
    std::string trackerType = trackerTypes[2];

    std::vector<cv::Rect> drones;// list of drones
    cv::Ptr<cv::Tracker> tracker;//create a tracker object
    cv::Mat frame; //holds video frame
    cv::Rect2d bbox;//create bounding box
    
    createTracker(tracker, trackerType);//bound tracker

    //choose which type of video to show
    	cv::VideoCapture video(vid);

    //Exit if video is not opened
    if(!video.isOpened())
    {
        std::cout << "Could not read video file" << std::endl;
        return 1;
    }
    
    //take first frame
    video >> frame;

    //draw a region of interest to track
    bbox = selectROI(frame,false);
 
    //draw an actual box on screen
    rectangle(frame, bbox, cv::Scalar(225, 0, 0), 2, 1);
    //init tracker
    tracker->init(frame, bbox);

    while(video.read(frame)) 
    {
        //Update tracking result
        bool ok = tracker->update(frame, bbox);

        if(ok)
        {
            //Tracking success: draw tracked object
            rectangle(frame, bbox, cv::Scalar(225, 0, 0), 2, 1);
        }

        else
        {
	    //show user tracking failure
            cv::putText(frame, "Tracking Failure Detected", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,225),2);
        }

        // Display frame.
        cv::imshow("Tracking", frame);

        // Exit if ESC pressed.
        if(cv::waitKey(1) == 27)
        {
            break;
        }
    }
    return 0;
}

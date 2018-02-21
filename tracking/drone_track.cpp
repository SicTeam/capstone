//This file will hold the inner workings of any tracking feature


#include "drone.h"



//Task:
//Input:
//Output:
Track::Track()
{
    //cascade_name = "/home/cody/Desktop/test_train/data/cascade.xml";
    min_neighbors = 7; //the higher this number the more strict detection is
    cascade_name = "cascade.xml";
    if(!cascade.load(cascade_name))
    {
        std::cout << "FAIL" << std::endl;
    }
    //cam2_detect = false; 
}



//Task:
//Input:
//Output:
Track::Track(std::string file_name)
{
    min_neighbors = 2;//the higher this number the more strict detection is
    cascade_name = file_name;
    if(!cascade.load(cascade_name))
    {
        std::cout << "FAIL" << std::endl;
    }

    cam2_detect = false; 
}



//Task: Detects drone from test images
//Input:
//Output:
int Track::detect_image()
{
    //String image("test/test_image/1.jpeg");
    cv::VideoCapture video("test/Video_1.avi");
    std::vector<cv::Rect> drones;
    cv::Mat frame;

    video >> frame;
    //frame = imread( image, IMREAD_COLOR);

    if(!detect(drones, frame))
    {
        std::cout << "No object Detected" << std::endl;
        return 0;
    }

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
    cv::Mat frame_gray;

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
    std::cout << "Objects Detected: " << size << std::endl;

    //cv::Point center(drones[0].x + drones[0].width/2, drones[0].y + drones[0].height/2);
    
    //set class target point to pass to pursuit
    //target = center;

    //XXX determine which element of drones is actually the drone
    for(size_t i = 0; i < size; ++i)
    {
        rectangle(frame, drones[i], cv::Scalar(225,0,0),2,8);
    }

    return size;
}


void Track::createTracker(cv::Ptr<cv::Tracker>& tracker, const std::string& trackerType) {

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

//XXX No tracking failure so it doesn't detect
//Task:   Allows tracking of object through video passed in
//Input:  Takes in video stored in testing directory
//Output: Displays video frame by frame while tracking selected object
int Track::kcf(char * vid)
{
    std::string trackerTypes[5] = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW"};
    std::string trackerType = trackerTypes[2];
    std::vector<cv::Rect> drones;
    cv::Ptr<cv::Tracker> tracker;
    cv::Mat frame; //holds video frame
    cv::Rect2d bbox;
    cv::Rect2d origin_box;
    bool trackFail = false;
    
    createTracker(tracker, trackerType);

    //Read video from a video clip
    cv::VideoCapture video(vid);

    //Exit if video is not opened
    if(!video.isOpened())
    {
        std::cout << "Could not read video file" << std::endl;
        return 1;
    }
    
    //take first frame
    video >> frame;

    //detect object in frame
    if(!detect(drones, frame))
    {
        std::cout << "--(!)No Initial Objects Detected" << std::endl;
        return 0;
    }

    bbox = drones[0];
    origin_box.x = bbox.x ; origin_box.y = bbox.y; origin_box.width = bbox.width; origin_box.height = bbox.height; 
    rectangle(frame, bbox, cv::Scalar(225, 0, 0), 2, 1);
    tracker->init(frame, bbox);

    //TEST Store bounding box and center
    cv::Point center(drones[0].x + drones[0].width/2, drones[0].y + drones[0].height/2);
    target_point = center;
    target = drones[0];
    display();
    //***************************************
    
    while(video.read(frame)) 
    {
        //Display all detections*****************
//        detect(drones, frame);
//        for(size_t i = 0; i < drones.size(); ++i)
//        {
//            rectangle(frame, drones[i], cv::Scalar(225,0,0),2,8);
//        }
        //***************************************

        if(trackFail)
        {
            cv::putText(frame, "Tracking re-initting", cv::Point(300, 100), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,225),2);
            createTracker(tracker, trackerType);
            tracker->init(frame, bbox);
            rectangle(frame, bbox, cv::Scalar(225, 0, 0), 2, 1);
            trackFail = false;
        }

        //Start timer
        double timer = (double)cv::getTickCount();
       
        //XXX Always returns true????
        //Update tracking result
        bool ok = tracker->update(frame, bbox);

	if(bbox.x < 50 || bbox.x > (frame.rows - 50) || bbox.y < 50 || bbox.y > (frame.cols - 50))
		ok = false;
        
        //Calculate Frames per second
        float fps = cv::getTickFrequency() / ((double)cv::getTickCount() - timer);

        if(ok)
        {
            //Tracking success: draw tracked object
            rectangle(frame, bbox, cv::Scalar(225, 0, 0), 2, 1);
        }
        else
        {
            //Tracking failure
            cv::putText(frame, "Tracking Failure Detected", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,225),2);
            
            if(!detect(drones, frame))
            {
                std::cout << "--(!)Lost Target" << std::endl;
                //return 0;
            }
            else
            {
                cv::putText(frame, "Found the object", cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,225),2);
                bbox = drones[0];
                cv::putText(frame, "Center - x: " + SSTR(int(bbox.x)) + "  y:" + SSTR(int(bbox.y)) , cv::Point(300, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,225),2);
                trackFail = true;
            } 
        }

	//show the scale difference between original box vs current box
        cv::putText(frame, "Scale: " + SSTR(double((bbox.width * bbox.height) / (origin_box.width * origin_box.height) *100 )) + "%", cv::Point(400,20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);
        
        // Display tracker type on frame
        cv::putText(frame, trackerType + " Tracker", cv::Point(100,20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);

        // Display FPS on frame
        cv::putText(frame, "FPS : " + SSTR(int(fps)), cv::Point(100,50), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50), 2);

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



//Task:   This function tracks using bounding box via video stream
//Input:  Receives camera feed from local device
//Output: Outputs video frame by frame showing tracking of object
int Track::kcf()
{
    std::string trackerType = "MEDIANFLOW";
    //std::string trackerType = "KCF";
    std::vector<cv::Rect> faces;   
    cv::Ptr<cv::Tracker> tracker;
    cv::Mat frame; //holds video frame
    cv::Rect2d bbox;
    cv::Rect2d origin_box;

 
    bool trackFail = false;

    createTracker(tracker, trackerType);
    
    //Read video from a camera
    cv::VideoCapture video(0);

    //Exit if video is not opened
    if(!video.isOpened())
    {
        std::cout << "--(!)Could not read video" << std::endl;
        return 1;
    }

    //second camera code
    cv::Mat frame2; 
    std::vector<cv::Rect> faces2;   
    
    bool second_cam = true; //tell if the second cam is on/off

    cv::VideoCapture video2(1);
    if(!video2.isOpened())
    {
	second_cam = false;
        std::cout << "--(!)Could not read video2 or not having 2nd camera" << std::endl;
       // return 1;
    }
    if(second_cam)
    	video2 >> frame2;

    //Read first frame
    video >> frame;
    
    //detect object in frame
    if(!detect(faces, frame))
    {
        std::cout << "--(!)No Initial Objects Detected" << std::endl;
        return 0;
    }

    //from the detect function, we get face object(s), point the bbox to first one
    bbox = faces[0];

    //compare the side of bounding box, however it only work with MEDIANFLOW tracker.
    origin_box.x = bbox.x ; origin_box.y = bbox.y; origin_box.width = bbox.width; origin_box.height = bbox.height; 
    rectangle(frame, bbox, cv::Scalar(225, 0, 0), 2, 1);
    tracker->init(frame, bbox);
    
    //std::cout << "Width :" << frame.cols << " Height: " << frame.rows  << std::endl;

    while(video.read(frame))
    {
        if(trackFail)
        {
            cv::putText(frame, "Tracking re-initting", cv::Point(300, 100), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,225),2);
            createTracker(tracker, trackerType);
            tracker->init(frame, bbox);
            rectangle(frame, bbox, cv::Scalar(225, 0, 0), 2, 1);
            trackFail = false;
        }

        //Start timer
        double timer = (double)cv::getTickCount();
        
        //Update tracking result
        bool ok = tracker->update(frame, bbox);


	if(bbox.x < 50 || bbox.x > (frame.rows - 50) || bbox.y < 50 || bbox.y > (frame.cols - 50))
		ok = false;
        
        //Calculate Frames per second
        float fps = cv::getTickFrequency() / ((double)cv::getTickCount() - timer);

        if(ok)
        {
            //Tracking success: draw tracked object
            rectangle(frame, bbox, cv::Scalar(225, 0, 0), 2, 1);
            cv::putText(frame, "x: " + SSTR(int(bbox.x)) + "  y:" + SSTR(int(bbox.y)) , cv::Point(300, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,225),2);
        }

        else
        {
            //Tracking failure
            cv::putText(frame, "Tracking Failure Detected", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,225),2);
            
            if(!detect(faces, frame))
            {
                std::cout << "--(!)Lost Target" << std::endl;
                //return 0;
            }        
            else
            {
                cv::putText(frame, "Found the object", cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,225),2);
                bbox = faces[0];
                cv::putText(frame, "Center - x: " + SSTR(int(bbox.x)) + "  y:" + SSTR(int(bbox.y)) , cv::Point(300, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,225),2);
                trackFail = true;
            }
        }

	//show the scale difference between original box vs current box
        cv::putText(frame, "Scale: " + SSTR(double((bbox.width * bbox.height) / (origin_box.width * origin_box.height) *100 )) + "%", cv::Point(400,20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);
	
            
        // Display tracker type on frame
        cv::putText(frame, trackerType + " Tracker", cv::Point(100,20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);

        // Display FPS on frame
        cv::putText(frame, "FPS : " + SSTR(int(fps)), cv::Point(100,50), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50), 2);

        // Display frame.
        cv::imshow("Tracking", frame);

	if(second_cam)
	{
		video2.read(frame2);
	
            	if(detect(faces2, frame2))
            	{
			cam2_detect = true;	
                	std::cout << "--Second camera found the target. Cam2_detect value : " << cam2_detect << std::endl;
            	}        
		
		else
		{
			cam2_detect = false;	
		}
        cv::imshow("Second cam detection", frame2);
	}

        // Exit if ESC pressed.
        if(cv::waitKey(1) == 27)
        {
            break;
        }
    }
    
    return 0;
}



//Task:
//Input:
//Output:
void Track::display()
{
    std::cout << "\n-----Tracking Location data-----" << std::endl;
    std::cout << "Coordinate X: " << target.x << std::endl;
    std::cout << "Coordinate Y: " << target.y << std::endl;
    std::cout << "Width: " << target.width << std::endl;
    std::cout << "Height: " << target.height << std::endl << std::endl;
    std::cout << "Point Center: (" << target_point.x << ", " << target_point.y << ")" << std::endl;
    std::cout << "-----------------------------------" << std::endl << std::endl;
    
}

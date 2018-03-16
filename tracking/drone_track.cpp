//This file will hold the inner workings of any tracking feature


#include "drone.h"



//Task:
//Input:
//Output:
Track::Track()
{
    cascade_name = "sim_cascade.xml";
    min_neighbors = 15; //the higher this number the more strict detection is
    //cascade_name = "cascade.xml";
    if(!cascade.load(cascade_name))
    {
        std::cout << "FAIL" << std::endl;
    }
    //cam2_detect = false; 
    image_center.x = 400;
    image_center.y = 400;
}



//Task:
//Input:
//Output:
Track::Track(std::string file_name)
{
    min_neighbors = 3;//the higher this number the more strict detection is
    cascade_name = file_name;
    if(!cascade.load(cascade_name))
    {
        std::cout << "FAIL" << std::endl;
    }

    cam2_detect = false; 
    //these are set to default values for our ROS camera feeds center location
    //XXX these would need to be changed or set to the correct center position 
    //based on size of images.
    image_center.x = 400;
    image_center.y = 400;
}



//Task: Detects drone from test images
//Input:
//Output:
int Track::detect_image(std::string image)
{
    //cv::VideoCapture video("test/Video_1.avi");
    std::vector<cv::Rect> drones;
    cv::Mat frame;

    //video >> frame;
    frame = cv::imread( image, cv::IMREAD_COLOR);

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
    //std::cout << "Objects Detected: " << size << std::endl;
    //if(size > 0)
      cv::Point center(drones[0].x + drones[0].width/2, drones[0].y + drones[0].height/2);
    
    //set class target point to pass to pursuit
    //target = center;

    //XXX determine which element of drones is actually the drone
//    for(size_t i = 0; i < size; ++i)
//    {
//        rectangle(frame, drones[i], cv::Scalar(225,0,0),2,8);
//    }
    rectangle(frame, drones[0], cv::Scalar(225,0,0),2,8);
    circle(frame, center, 1, cv::Scalar(0,0,225), 2, 1, 0);

    return 1;
}


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

//XXX No tracking failure so it doesn't detect
//Task:   Allows tracking of object through video passed in
//Input:  Takes in video stored in testing directory
//Output: Displays video frame by frame while tracking selected object
int Track::kcf(char * vid)
{
    std::string trackerTypes[5] = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW"};
    std::string trackerType = trackerTypes[4];
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
        trackFail = true;
    }

    bbox = drones[0];
    origin_box.x = bbox.x ; origin_box.y = bbox.y; origin_box.width = bbox.width; origin_box.height = bbox.height; 
    rectangle(frame, bbox, cv::Scalar(225, 0, 0), 2, 1);
    tracker->init(frame, bbox);

    //TEST Store bounding box and center
    //XXX don't use drones[0] to calculate center point, use bbox instead
    //caz it drones[0] ONLY update when we use detect
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
            rectangle(frame, bbox, cv::Scalar(225, 0, 0), 2, 1);
            tracker->init(frame, bbox);
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
            trackFail = true;
            //Tracking failure
            cv::putText(frame, "Tracking Failure Detected", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,225),2);
            trackFail = true;

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

//Task:   This function tracks using bounding box via 2 video streams
//Input:  Receives camera feed from 2 videos
//Output: Outputs video frame by frame showing tracking of object
int Track::kcf(char * vid1 , char * vid2)
{
    std::string trackerTypes[5] = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW"};
    std::string trackerType = trackerTypes[4];

    //left tracker init
    std::vector<cv::Rect> drones_left;
    cv::Ptr<cv::Tracker> tracker_left;
    cv::Mat frame_left; //holds video frame
    cv::Rect2d bbox_left;
    cv::Rect2d origin_box_left;
    bool trackFail_left = false;
    bool left_cond = true; // to signal if the left camera still running

    //right tracker init
    std::vector<cv::Rect> drones_right;
    cv::Ptr<cv::Tracker> tracker_right;
    cv::Mat frame_right; //holds video frame
    cv::Rect2d bbox_right;
    cv::Rect2d origin_box_right;
    bool trackFail_right = false;
    bool right_cond = true;
    
    //tracker init
    createTracker(tracker_left, trackerType);
    createTracker(tracker_right, trackerType);

    //Read video from a video clip
    cv::VideoCapture video_left(vid1);
    cv::VideoCapture video_right(vid2);

    //Exit if video is not opened
    if(!video_left.isOpened())
    {
        std::cout << "Could not read left video file" << std::endl;
	return 1;
    }
    
    if(!video_right.isOpened())
    {
        std::cout << "Could not read right video file" << std::endl;
	return 1;
    }

    //take first frame
    video_left >> frame_left;
    video_right >> frame_right;

    //detect object in frame
    if(!detect(drones_left, frame_left))
    {
        std::cout << "--(!)No Initial Objects Detected" << std::endl;
        return 0;
    }

    //detect object in frame
    if(!detect(drones_right, frame_right))
    {
        std::cout << "--(!)No Initial Objects Detected" << std::endl;
        return 0;
    }

    bbox_left = drones_left[0];
    origin_box_left.x = bbox_left.x ; origin_box_left.y = bbox_left.y; origin_box_left.width = bbox_left.width; origin_box_left.height = bbox_left.height; 
    rectangle(frame_left, bbox_left, cv::Scalar(225, 0, 0), 2, 1);
    tracker_left->init(frame_left, bbox_left);
    
    bbox_right = drones_right[0];
    origin_box_right.x = bbox_right.x ; origin_box_right.y = bbox_right.y; origin_box_right.width = bbox_right.width; origin_box_right.height = bbox_right.height; 
    rectangle(frame_right, bbox_right, cv::Scalar(225, 0, 0), 2, 1);
    tracker_right->init(frame_right, bbox_right);
    
    while(frame_left.data || frame_right.data)
    {
	video_left.read(frame_left);
	video_right.read(frame_right);

	if(!frame_left.data)
	{
		//std::cout << "Left video is stopped" << std::endl;
		left_cond = false;
	}

	if(!frame_right.data)
	{
		//std::cout << "Right video is stopped" << std::endl;
		right_cond = false;
	}

	if(left_cond)
	{
		if(trackFail_left) //re-init when left cam fail to track
		{
		    cv::putText(frame_left, "Tracking re-initting", cv::Point(300, 100), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,225),2);
		    createTracker(tracker_left, trackerType);
		    tracker_left->init(frame_left, bbox_left);
		    rectangle(frame_left, bbox_left, cv::Scalar(225, 0, 0), 2, 1);
		    trackFail_left = false;
		}

		bool ok_left = tracker_left->update(frame_left, bbox_left);

		if(bbox_left.x < 50 || bbox_left.x > (frame_left.rows - 50) || bbox_left.y < 50 || bbox_left.y > (frame_left.cols - 50))
			ok_left = false;


		//procedure for left cam
		if(ok_left)
		{
		    //Tracking success: draw tracked object
		    rectangle(frame_left, bbox_left, cv::Scalar(225, 0, 0), 2, 1);
		}
		else
		{
		    trackFail_left = true;
		    //Tracking failure
		    cv::putText(frame_left, "Tracking Failure Detected", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,225),2);
		    
		    if(!detect(drones_left, frame_left))
		    {
			std::cout << "--(!)Left Camera Lost Target" << std::endl;
		    }
		    else
		    {
			cv::putText(frame_left, "Found the object", cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,225),2);
			bbox_left = drones_left[0];
		    } 
		}

		left_target_point.x = bbox_left.x + bbox_left.width/2;
		left_target_point.y = bbox_left.y + bbox_left.height/2;
		
		cv::putText(frame_left, "Scale: " + SSTR(double((bbox_left.width * bbox_left.height) / (origin_box_left.width * origin_box_left.height) *100 )) + "%", cv::Point(400,20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);
		//On LEFT cam
		//Display tracker type on frame
		cv::putText(frame_left, trackerType + " Tracker", cv::Point(100,20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);

		// Display left frame.
		if(frame_left.data)
			cv::imshow("Left camera", frame_left);
	}

	if(right_cond)
	{

		if(trackFail_right) //re-init when right cam fail to track
		{
		    cv::putText(frame_right, "Tracking re-initting", cv::Point(300, 100), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,225),2);
		    createTracker(tracker_right, trackerType);
		    tracker_right->init(frame_right, bbox_right);
		    rectangle(frame_right, bbox_right, cv::Scalar(225, 0, 0), 2, 1);
		    trackFail_right = false;
		}
	       
		//Update tracking result
		bool ok_right = tracker_right->update(frame_right, bbox_right);



		if(bbox_right.x < 50 || bbox_right.x > (frame_right.rows - 50) || bbox_right.y < 50 || bbox_right.y > (frame_right.cols - 50))
			ok_right = false;

		//procedure for right cam
		if(ok_right)
		{
		    //Tracking success: draw tracked object
		    rectangle(frame_right, bbox_right, cv::Scalar(225, 0, 0), 2, 1);
		}
		else
		{
		    trackFail_right = true;
		    //Tracking failure
		    cv::putText(frame_right, "Tracking Failure Detected", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,225),2);
		    
		    if(!detect(drones_right, frame_right))
		    {
			std::cout << "--(!)Right Camera Lost Target" << std::endl;
		    }
		    else
		    {
			cv::putText(frame_right, "Found the object", cv::Point(100, 100), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0,0,225),2);
			bbox_right = drones_right[0];
		    } 
		}
		//calculate target coordinate on right screen
		right_target_point.x = bbox_right.x + bbox_right.width/2;
		right_target_point.y = bbox_right.y + bbox_right.height/2;

		//show the scale difference between original box vs current box
		cv::putText(frame_right, "Scale: " + SSTR(double((bbox_right.width * bbox_right.height) / (origin_box_right.width * origin_box_right.height) *100 )) + "%", cv::Point(400,20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);

		// Display tracker type on frame
		cv::putText(frame_right, trackerType + " Tracker", cv::Point(100,20), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(50,170,50),2);

		// Display right frame.
		if(frame_right.data)
			cv::imshow("Right camera", frame_right);
	}


	if(left_cond && right_cond)
	{
		//only calculate the final center point when left and right camera is opened

		target_point.x = (left_target_point.x + right_target_point.x) / 2;
		target_point.y = (left_target_point.y + right_target_point.y)/2;
		//std::cout << "x: " << target_point.x << " y: " << target_point.y  << std::endl;
	}

	else if(left_cond) //if only left camera work
	{
		//only calculate the final center point when left and right camera is opened

		target_point.x = left_target_point.x;
		target_point.y = left_target_point.y;
		//std::cout << "x: " << target_point.x << " y: " << target_point.y  << std::endl;
	}

	else if(right_cond) //if only right camera work
	{
		//only calculate the final center point when left and right camera is opened

		target_point.x = right_target_point.x;
		target_point.y = right_target_point.y;
		//std::cout << "x: " << target_point.x << " y: " << target_point.y  << std::endl;
	}

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

    int frame_count = 0;

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

    frame.copyTo(mat_store[0]);	
    frame.copyTo(mat_store[1]);

    while(video.read(frame))
    {
	frame.copyTo(mat_store[0]);	

	if(frame_count % 36 == 35)
	{
		mat_store[0].copyTo(mat_store[1]);
		target_store.x = target_point.x;
		target_store.y = target_point.y;
	}

        cv::imshow("35 frames before", mat_store[1]);

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
            trackFail = true;
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

	target_point.x = bbox.x; target_point.y = bbox.y;

	frame_count += 1 ;

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

void Track::normalize() //XXX might need to create this in ros_node or pass a signal for normalization
{

}
/*XXX produced in ros_node.cpp
void Track::pursuit()//TAKE TWO STRUCTS WITH IMAGES AND BBOX's FOR DRONES
{
   geometry_msgs::PoseWithCovariance sig;
   sig.pose.position.x = depth(left_c, right_c);
   
}

*/
//Task: provides a depth to target drone 
//Input: center points from the detected drone, from both L & R images
//Output: integer value Z for estimated depth
int Track::depth(cv::Point left, cv::Point right)
{
   return abs(left.x - right.x);
}

//Task: tranform image detection center into projected (x,y) value for drone movement
//Input: the (x,y) values provided by the left image detection/tracking of proper target
//Output: projected CV::Point object
cv::Point Track::projectedXY(cv::Point drone_center)
{
   //XXX for drone control inputs:
   //    proj.x is Left(+) Right(-), proj.y is Up(+) Down(-)
   cv::Point proj;
   //use image_center with drone_center t0 product the correctly projected values
   //currently these will produce TRUE values related to the 
   //image & may need to be scaled down by a factor
   proj.x = image_center.x - drone_center.x;
   proj.y = image_center.y - drone_center.y;
   return proj;
}

//Task:
//Input:
//Output:
int Track::track(cv::Mat & frame, cv::Rect & drone, cv::Ptr<cv::Tracker> & tracker)
{
    std::string trackerTypes[5] = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW"};
    std::string trackerType = trackerTypes[4];
	 
    //tracker init
    cv::Rect2d bbox = drone;
    //cv::Rect2d origin_box;
    bool trackFail = false;
    bool cond = true; // to signal if the left camera still running
    bool ok = false;

    if(!tracker)
    {
    	createTracker(tracker, trackerType);
	    //tracker->init(frame, drone);
        //rectangle(frame, drone, cv::Scalar(225, 0, 0), 2, 1);
    }
 
    //bbox = drone;
    //origin_box.x = bbox.x ; origin_box.y = bbox.y; origin_box.width = bbox.width; origin_box.height = bbox.height; 
    rectangle(frame, bbox, cv::Scalar(225, 0, 0), 2, 1);
    tracker->init(frame, bbox);

    ok = tracker->update(frame, bbox);
    
    if(bbox.x < 50 || bbox.x > (frame.rows - 50) || bbox.y < 50 || bbox.y > (frame.cols - 50))
		ok = false;


    if(ok)
    {
        //Tracking success: draw tracked object
        cv::Point center(drone.x + drone.width/2, drone.y + drone.height/2);
        rectangle(frame, bbox, cv::Scalar(225, 0, 0), 2, 1);
	    return 1;
    }
    
    return 0;
} 
//Task:
//Input:
//Output:

int Track::track_tester(char * vid)
{
    std::string trackerTypes[5] = {"BOOSTING", "MIL", "KCF", "TLD", "MEDIANFLOW"};
    std::string trackerType = trackerTypes[4];
    std::vector<cv::Rect> drones;
    cv::Ptr<cv::Tracker> tracker;
    cv::Mat frame; //holds video frame
    cv::Rect2d bbox;
    
    createTracker(tracker, trackerType);


    #if (vid == NULL)
    	cv::VideoCapture video(0);
    #else
    	cv::VideoCapture video(vid);
    #endif

    //Exit if video is not opened
    if(!video.isOpened())
    {
        std::cout << "Could not read video file" << std::endl;
        return 1;
    }
    
    //take first frame
    video >> frame;

    bbox = selectROI(frame,false);

    rectangle(frame, bbox, cv::Scalar(225, 0, 0), 2, 1);
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

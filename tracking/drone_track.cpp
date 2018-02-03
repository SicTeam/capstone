//This file will hold the inner workings of any tracking feature


#include "drone.h"



//Task:
//Input:
//Output:
Track::Track()
{

}



//Task:
//Input:
//Output:
Track::~Track()
{

}

int Track::detect(int &x,int &y,int &width, int &height)
{

	int foundObj = 0;

	String drone_cascade_name = "face.xml";//swap with the drone training data
	CascadeClassifier drone_cascade;
	Mat frame;

        if( !drone_cascade.load( drone_cascade_name ) )
	{ 
		cout <<"--(!)Error loading drone cascade\n" << endl; 
		return 1;
       	}

	VideoCapture video_2(0);

    	//Read first frame
    	video_2 >> frame;
	
	while(video_2.read(frame))
	{

        	if( frame.empty() )
        	{
            	cout << " --(!) No captured frame -- Break!" << endl;
            	break;
        	}

		//function from here

    		std::vector<Rect> drones;
    		Mat frame_gray;

    		cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    		equalizeHist( frame_gray, frame_gray );

    		//-- Detect drones
    		drone_cascade.detectMultiScale( frame_gray, drones, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );

		imshow("Detect",frame);

		if(drones.size() == 0)
		{
			cout << "Can't find the object" << endl;
		        break;	
		}

		else
		{
    			x = drones[0].x; 
    			y = drones[0].y;
   			width = drones[0].width;
    			height = drones[0].height;
			video_2.release();
			if(!video_2.isOpened())
			{
				cout<<"Video is closed in detection" << endl;
			}
			return 1;
		}

        	char c = (char)waitKey(10);
        	if( c == 27 ) { break; } // escape
	}

	video_2.release();
	if(!video_2.isOpened())
	{
		cout<<"Video is closed in detection" << endl;
	}
	return 0;
}

//Task:
//Input:
//Output:
int Track::detect()
{
    return 0;
}

//Task:   Allows tracking of object through video passed in
//Input:  Takes in video stored in testing directory
//Output: Displays video frame by frame while tracking selected object
int Track::kcf(char * vid)
{
    //create tracker
    string trackerType = "KCF";
    //string video_cap = vid;
    Ptr<Tracker> tracker;
    Mat frame; //holds video frame

    #if (CV_MINOR_VERSION < 3)
    {
        tracker = Tracker::create(trackerType);
    }
    #else
    {
        tracker = TrackerKCF::create();
    }
    #endif

    int x,y,width,height,getInfo;//parameter for the box

    /*getInfo = detect(x,y,width,height);
    if(getInfo == 0)
    	return 1;
    cout << "Found the para for object:  " << x << "  " << y << "  " << width <<"  " << height << endl;
    */

    //Read video from a video clip
    VideoCapture video(vid);

    //Exit if video is not opened
    if(!video.isOpened())
    {
        cout << "Could not read video file" << endl;
        return 1;
    }

    //Read first frame
    video >> frame;
    
    //Define initial bounding box
    Rect2d bbox(150, 50, 100, 300);
    //Rect2d bbox(x,y,width,height);

    if(bbox.width==0 || bbox.height==0)
        return 1;

    //comment this out to not choose the box with mouse
    bbox = selectROI(frame, false);
    tracker->init(frame, bbox);

    while(video.read(frame)) 
    {
        //Start timer
        double timer = (double)getTickCount();
        
        //Update tracking result
        bool ok = tracker->update(frame,bbox);
        
        //Calculate Frames per second
        float fps = getTickFrequency() / ((double)getTickCount() - timer);

        if(ok)
        {
            //Tracking success: draw tracked object
            rectangle(frame, bbox, Scalar(225, 0, 0), 2, 1);
            //(x+1/2 height, x + 1/2 width) for middle of rectangle
        }
        else
        {
            //Tracking failure
            putText(frame, "Tracking Failure Detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,225),2);
        }
        
        // Display tracker type on frame
        putText(frame, trackerType + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);

        // Display FPS on frame
        putText(frame, "FPS : " + SSTR(int(fps)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);

        // Display frame.
        imshow("Tracking", frame);

        // Exit if ESC pressed.
        if(waitKey(1) == 27)
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
    //create tracker
    string trackerType = "MEDIANFLOW";
    Ptr<Tracker> tracker;
    String drone_cascade_name = "face.xml";//swap with the drone training data
    CascadeClassifier drone_cascade;
    Mat frame; //holds video frame
    bool trackFail = false;

    if( !drone_cascade.load( drone_cascade_name ) )
    { 
	cout <<"--(!)Error loading drone cascade\n" << endl; 
	return 1;
    }

    #if (CV_MINOR_VERSION < 3)
    {
        tracker = Tracker::create(trackerType);
    }
    #else
    {
        tracker = TrackerKCF::create();
    }
    #endif

    //Define initial bounding box

    int x,y,width,height,getInfo;//parameter for the box

    getInfo = detect(x,y,width,height);
    if(getInfo == 0)
    	return 1;
    //cout << "Found the para for object:  " << x << "  " << y << "  " << width <<"  " << height << endl;

    //Read video from a camera
    VideoCapture video(0);

    //Exit if video is not opened
    if(!video.isOpened())
    {
        cout << "Could not read video file" << endl;
        return 1;
    }

    //Read first frame
    video >> frame;

    Rect2d bbox(x,y,width,height);

    //Rect2d bbox(287, 23, 86, 320);

    if(bbox.width==0 || bbox.height==0)
        return 1;

    //bbox = selectROI(frame, false);
    tracker->init(frame, bbox);

    while(video.read(frame)) 
    {

	Rect2d new_bbox(bbox.x,bbox.y,bbox.width,bbox.height);

	if(trackFail)
	{
            	putText(frame, "Tracking re-initting", Point(300, 100), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,225),2);
    		#if (CV_MINOR_VERSION < 3)
    		{
       	 		tracker = Tracker::create(trackerType);
    		}
    		#else
    		{
        		tracker = TrackerKCF::create();
    		}
    		#endif

		tracker->init(frame,new_bbox);
		trackFail = false;
	}

        //Start timer
        double timer = (double)getTickCount();
        
        //Update tracking result
        bool ok = tracker->update(frame,new_bbox);
        
        //Calculate Frames per second
        float fps = getTickFrequency() / ((double)getTickCount() - timer);

        if(ok)
        {
            //Tracking success: draw tracked object
            rectangle(frame, new_bbox, Scalar(225, 0, 0), 2, 1);
            //(x+1/2 height, x + 1/2 width) for middle of rectangle
        }

        else
        {
            //Tracking failure
            putText(frame, "Tracking Failure Detected", Point(100, 80), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,225),2);
		
    	    std::vector<Rect> drones;
    	    Mat frame_gray;

    	    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    	    equalizeHist( frame_gray, frame_gray );

    		//-- Detect drones
    	    drone_cascade.detectMultiScale( frame_gray, drones, 1.1, 1, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );
	    if(drones.size() == 0)
	    {
                putText(frame, "Can't find the object", Point(100, 100), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,225),2);
	    }

	    else
	    {
                putText(frame, "Found the object", Point(100, 100), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0,0,225),2);
	    	bbox.x = drones[0].x;  			bbox.y = drones[0].y;
	    	bbox.width = drones[0].width; 		bbox.height = drones[0].height;

                putText(frame, "Center - x: " + SSTR(int(bbox.x)) + "  y:" + SSTR(int(bbox.y)) , Point(300, 50), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,0,225),2);
		trackFail = true;
	    }
        }
        
        // Display tracker type on frame
        putText(frame, trackerType + " Tracker", Point(100,20), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50),2);

        // Display FPS on frame
        putText(frame, "FPS : " + SSTR(int(fps)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);

        // Display frame.
        imshow("Tracking", frame);

        // Exit if ESC pressed.
        if(waitKey(1) == 27)
        {
            break;
        }
    }
    return 0;
}


//Task:detect the drone from a live video input
//Input:any video (with or without drone)
//Output:successfully detect the drone from the video stream
int Track::test()
{
	String drone_cascade_name = "face.xml";//swap with the drone training data
	CascadeClassifier drone_cascade;
	Mat frame;

        if( !drone_cascade.load( drone_cascade_name ) )
	{ cout <<"--(!)Error loading drone cascade\n" << endl; 
		return 1;
       	};

	//VideoCapture video(0);
	VideoCapture video("test/0001.webm");

    	if(!video.isOpened())
    	{
        	cout << "Could not read video file" << endl;
        	return 1;
    	}

    	//Read first frame
    	video >> frame;
	
	while(video.read(frame))
	{
        	if( frame.empty() )
        	{
            	cout << " --(!) No captured frame -- Break!" << endl;
            	break;
        	}

	//function from here

    std::vector<Rect> drones;
    Mat frame_gray;

    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );

    //-- Detect drones
    drone_cascade.detectMultiScale( frame_gray, drones, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );

    for ( size_t i = 0; i < drones.size(); i++ )
    {
        Point center( drones[i].x + drones[i].width/2, drones[i].y + drones[i].height/2 );
        ellipse( frame, center, Size( drones[i].width/2, drones[i].height/2 ), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );

        //Mat droneROI = frame_gray(drones[i]);
    }

    //-- Show what you got
    imshow( "Drone detection", frame );

	//to here

        if( waitKey(1) == 27 ) { break; } // escape
	}

	return 0;
}


//Task:
//Input:
//Output:
void Track::display()
{

    cout << "Tracking" << endl;
}

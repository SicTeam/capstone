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


//XXX Next task is to track but with live stream
//Task:
//Input:
//Output:
int Track::kcf(char * vid)
{
    //create tracker
    string trackerType = "KCF";
    string video_cap = vid;
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
    //Read video
    VideoCapture video(video_cap);

    //Exit if video is not opened
    if(!video.isOpened())
    {
        cout << "Could not read video file" << endl;
        return 1;
    }

    //Read first frame
    video >> frame;
    
    //Define initial bounding box
    Rect2d bbox(287, 23, 86, 320);

    if(bbox.width==0 || bbox.height==0)
        return 1;

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
            //(x-1/2 height, x + 1/2 width) for middle of rectangle
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



//Task:
//Input:
//Output:
void Track::display()
{

    cout << "Tracking" << endl;
}

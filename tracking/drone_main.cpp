//this file has the control flow of both tracking and detection


#include "drone.h"

//XXX will not be calling this from control_flow in upper directory. this will 
//XXX only be used for testing purposes. Track class should be treated as a library possibly
//XXX or we pass bounding box info to pursuit from tracking function

//Task:
//Input:
//Output:
int main(int argc, char ** argv)
{
    Track drone_track;
    
    std::cout << "OpenCV Version: " << CV_VERSION << std::endl;
    std::cout << "OpenCV Minor Version: " << CV_MINOR_VERSION << std:: endl;
    //for testing the detection
//    drone_track.detect_image();

    drone_track.detect_image(argv[1]);

    //detect/track off video passed in, else use live video feed
    if(argv[1])
    {
        drone_track.kcf(argv[1]);
    }
    else
    {
        Track face_track("face.xml");
        face_track.kcf();
    }
    
    return 0;
}

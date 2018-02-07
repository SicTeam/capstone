//this file has the control flow of both tracking and detection


#include "drone.h"



//Task:
//Input:
//Output:
int main(int argc, char ** argv)
{
    Track drone_track;
    
    std::cout << "OpenCV Version: " << CV_VERSION << std::endl;

    //for testing the detection
//    drone_track.detect_image();

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

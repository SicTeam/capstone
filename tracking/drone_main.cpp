//this file has the control flow of both tracking and detection


#include "drone.h"



//Task:
//Input:
//Output:
int main(int argc, char ** argv)
{
    Track track;
    
    cout << "OpenCV Version: " << CV_VERSION << endl;
    cout << "opencv minor: " << CV_MINOR_VERSION << endl;
    //if video is passed in then use it, otherwise use video stream
    //track.test();

    //for testing the detection
    track.detect();

    if(argv[1]){
//        track.kcf(argv[1]);
    }
    else{
  //      track.kcf();
    }
    
    return 0;
}

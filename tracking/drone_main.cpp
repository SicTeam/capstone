//this file has the control flow of both tracking and detection


#include "drone.h"



//Task:
//Input:
//Output:
int main(int argc, char ** argv)
{
    Track track;
    cout << "OpenCV Version: " << CV_VERSION << endl;

    track.detect();
    //if video is passed in then use it, otherwise use video stream
    if(argv[1]){
        //track.kcf(argv[1]);
    }
    else{
        //track.kcf();

        //track.display();
    }
    
    return 0;
}

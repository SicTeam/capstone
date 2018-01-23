//this file has the control flow of both tracking and detection


#include "drone.h"



//Task:
//Input:
//Output:
int main(int argc, char ** argv)
{
    Track track;
    Detect detect;

    cout << "OpenCV Version: " << CV_VERSION << endl;

    if(argv[1]){
        track.kcf(argv[1]);
    }
    else{
        track.kcf();
        //cout << "Begin" << endl;

        track.display();
        detect.display();
    }
    return 0;
}

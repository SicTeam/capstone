//this file has the control flow of both tracking and detection


#include "drone.h"



//Task:
//Input:
//Output:
int main(int argc, char ** argv)
{
    Track track;
    Detect detect;

    //VideoCapture video(0);	

    //track.kcf(argv[1]);

    track.kcf();

    //cout << "Begin" << endl;

    //cout <<"CV version" << CV_VERSION << endl;

    //track.display();
    //detect.display();

    return 0;
}

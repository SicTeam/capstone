//this file has the control flow of both tracking and detection


#include "drone.h"



//Task:
//Input:
//Output:
int main(int argc, char ** argv)
{
    Track track;
    Detect detect;

    track.kcf(argv[1]);
    //cout << "Begin" << endl;

    //track.display();
    //detect.display();

    return 0;
}

//this file has the control flow of both tracking and detection


#include "drone.h"



//Task:
//Input:
//Output:
int main(int argc, char ** argv)
{
    Track track;
    Detect detect;

<<<<<<< HEAD
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
=======
    //VideoCapture video(0);	

    //track.kcf(argv[1]);

    track.kcf();

    //cout << "Begin" << endl;

    //cout <<"CV version" << CV_VERSION << endl;

    //track.display();
    //detect.display();

>>>>>>> 0e785f4715da75af9112b198c486a8653b24edaa
    return 0;
}

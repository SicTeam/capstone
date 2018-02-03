//This file holds the class interface for the class that contains tracking
//features


#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>


#include "opencv2/objdetect.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <stdio.h>

using namespace cv;
using namespace std;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()



//Task:
class Track
{
    public:
        Track();
        ~Track();
        int detect();
        int kcf(char * vid);
        int kcf();
	int test();
        void display();
	int detect(int &x,int &y,int &width, int &height);

    private:
        
};

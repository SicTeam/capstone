//This file holds the class interface for the class that contains tracking
//features


#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

using namespace cv;
using namespace std;

// Convert to string
#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()


//Task:
class Detect
{
    public:
        Detect();
        ~Detect();
        void display();
    private:

};



//Task:
class Track
{
    public:
        Track();
        ~Track();
        int kcf(char * vid);
        void display();

    private:
        
};

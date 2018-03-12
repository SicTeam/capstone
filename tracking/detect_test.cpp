//this file will contain function call for testing purpose - objective : detect a drone from image
#include "drone.h"

int main(int argc, char ** argv)
{
    Track drone_track;

    if(argc > 1)    
    	drone_track.detect_image(argv[1]);
    else
	std::cout << "Need an image arguement followed after detect" << std::endl;
    return 0;
}

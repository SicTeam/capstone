#include "drone.h"

int main(int argc, char ** argv)

{
    Track drone_track;
    if(argc >= 1)
    	drone_track.track_tester(argv[1]);
    else
	std::cout << "Need an image arguement after ./track" << std::endl;
    return 0;
}

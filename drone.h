
#include <unistd.h>
#include <atomic>
//#include "../include/aruco.h"

class drone {
public:

	int leaderID;
	bool orientation; // true = right, false = left.
	double distanceForward;
	double distanceRightLeft;
	double distanceHeight;
	double angle;
	std::atomic<bool> commandFlag=false;

	drone(int leaderID, bool orientation, double distanceForward, double distanceRightLeft, double distanceHeight);

	drone();

	//~drone();

};

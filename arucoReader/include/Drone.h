
#include <unistd.h>
#include "../include/aruco.h"

class drone {
public:

	int leaderID;
	bool orientation; // true = right, false = left.
	double distanceForward;
	double distanceRightLeft;
	double distanceHeight;

	drone(int leaderID, bool orientation, double distanceForward, double distanceRightLeft, double distanceHeight);

	drone();

	~drone();

}

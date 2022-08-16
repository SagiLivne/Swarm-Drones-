
#include <unistd.h>
#include "../include/drone.h"

drone::drone(int leaderID, bool orientation, double distanceForward, double distanceRightLeft, double distanceHeight) {
	this->leaderID = leaderID;
	this->orientation = orientation;
	this->distanceForward = distanceForward;
	this->distanceRightLeft = distanceRightLeft;
	this->distanceHeight = distanceHeight;
	this->angle = 0;  // need to add leter TODO
}

drone::drone() {
	this->leaderID = 0;
	this->orientation = true;
	this->distanceForward = 0;
	this->distanceRightLeft = 0;
	this->distanceHeight = 0;
	this->angle = 0;
}


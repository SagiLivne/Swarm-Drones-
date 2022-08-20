//
// Created by tzuk on 6/6/22.
//
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <unistd.h>
#include <deque>
#include "aruco.h" //Might not need
#include "../include/drone.h"
#include <cmath>
#include <math>

#define FORWARD 40
#define RIGHT_LEFT 30 
#define HEIGHT 3 
#define BUFFER_SIZE 10
#define RADIUS 50
#define LIM_FORWARD 10
#define LIM_RIGHT_LEFT 5
//#define LIM_HEIGHT 5

const std::string movement = "3 ";
const std::string noMovement = "0 ";


std::deque<drone> buffer;
bool isLost = false;


// update drones movement with regards to leader.
void updateMovement(drone& drone, aruco& detector, ctello::Tello& tello) {
	std::string command = "rc ";

	if (drone.distanceRightLeft)
		command += movement;
	else
		command += noMovement;

	if (drone.distanceForward)
		command += movement;
	else
		command += noMovement;

	if (drone.distanceHeight)
		command += movement;
	else
		command += noMovement;

	if (drone.angle)
		command += movement;
	else
		command += noMovement;

	tello.sendCommand(command);

}


void circularAngle(drone& d1, double angle) {
	d1.distanceForward += RADIUS - RADIUS * std::cos(angle * M_PI / 180);
	d1.distanceRightLeft += RADIUS * std::sin(angle * M_PI / 180);
}


void distances(drone& drone, aruco& detector, ctello::Tello& tello) {
	if (detector.inFormation == true) {
		if (std::abs(detector.forward - FORWARD) > LIM_FORWARD)
			drone.distanceForward = detector.forward - FORWARD;
		else
			drone.distanceForward = 0;

		if (std::abs(std::abs(detector.rightLeft) - RIGHT_LEFT) > LIM_RIGHT_LEFT)
			drone.distanceRightLeft = std::abs(detector.rightLeft) - RIGHT_LEFT;
		else
			drone.distanceRightLeft = 0;

		drone.distanceHeight = std::abs(detector.upDown) > HEIGHT ? detector.upDown : 0;

		drone.angle = detector.leftOverAngle.first; // Maybe we shall add angle limit.
		isLost = false;
		circularAngle(drone, detector.leftOverAngle.first);

		//std::cout<<" drone: "<< std::endl;
		//std::cout<<"forward: "<< drone.distanceForward << " rightLeft :"<< drone.distanceRightLeft<< " height: "<<drone.distanceHeight<< " angle: "<< drone.angle  << std::endl;


		updateMovement(drone, detector, tello);
	}

}


//In case of not finding a leader, moving in the direction of the last frame captured.
void lostLeader(aruco& detector) {
	int i = 0;
	if (!isLost) {
		isLost = true;
		drone lastFrame = buffer.front();  // 
		distances(lastFrame, detector);  // 
	}
}


void runAruco(aruco& detector, drone& d1, ctello::Tello& tello) {
	int tmpId;

	tello.SendCommandWithResponse("takeoff");
	tello.SendCommandWithResponse("streamon");
	std::string videoPath = data["onlineVideoPath"];


	while (true) { //Multiplied by 100 in order to display it in cm.
	  /*std::cout << " detector: " << std::endl;
	  std::cout << "forward: " << detector.forward << " right left: " << detector.rightLeft << " updown: " << detector.upDown
				  << " angle: " << detector.leftOverAngle.first << " clockwise: " << detector.leftOverAngle.second <<"  ID: "<< detector.ID <<"  right or left: "<< detector.rightInForm<<std::endl;
	 */

		if (detector.ID != -1) {
			tmpId = detector.ID;
		}


		int i = 0;
		if (detector.ID == -1) {
			while (i < 5) {// timer for leader search when lost. after 5 seconds of not finding the leader stop searching and stop in place.
				lostLeader(detector);
				sleep(1);
				std::cout << "i=" << i << std::endl;
				i++;
				if (detector.ID != tmpId && detector.ID != -1)
					detector.init = true;
			}
			//while(detector.ID==-1);
			detector.init = true;
		}
		else {
			std::cout << "  detector" << detector.init << std::endl;
			distances(d1, detector);
		}
	}
}






//Adding new drone frame to the buffer.
void addToBuffer(drone& dr) {
	if (buffer.size() == BUFFER_SIZE) {
		drone d1 = buffer.back();
		buffer.pop_back();
	}
	buffer.push_front(dr);
}




// insert a drone into and existing formation
void insertToFormation() {

}

int main() {
	std::ifstream programData("../config.json");
	nlohmann::json data;
	programData >> data;
	programData.close();
	std::string yamlCalibrationPath = data["yamlCalibrationPath"];
	bool isCameraString = data["isCameraString"];
	float currentMarkerSize = data["currentMarkerSize"];

	drone d1;

	ctello::Tello tello;

	if (isCameraString) {
		std::string cameraString = data["cameraString"];
		aruco detector(yamlCalibrationPath, cameraString, currentMarkerSize);
		runAruco(detector, d1);
		tello.SendCommandWithResponse("streamon");
		std::string videoPath = data["onlineVideoPath"];
	}
	else {
		int cameraPort = data["cameraPort"];
		aruco detector(yamlCalibrationPath, cameraPort, currentMarkerSize);
		runAruco(detector, d1, tello);
	}
}

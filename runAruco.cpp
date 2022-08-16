//
// Created by tzuk on 6/6/22.
//
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <unistd.h>
#include <deque>
#include "aruco.h" //Might not need
#include "../include/drone.h"


#define FORWARD 60
#define RIGHT_LEFT 20 
#define HEIGHT 3 
#define BUFFER_SIZE 10

std::deque<drone> buffer;
bool isLost = false;


// update drones movement with regards to leader.
void updateMovement(drone& drone, aruco& detector) {


}

//In case of not finding a leader, moving in the direction of the last frame captured.
void lostLeader(aruco &detector) {
    int i=0;
    if (!isLost) {
        isLost = true;
    while(i<5){  // timer for leader search when lost. after 5 seconds of not finding the leader stop searching and stop in place.
	drone lastFrame = buffer.front();  // 
        updateMovement(lastFrame, detector);  // 
        sleep(1000);
        i++;      
    }
  }
}



void runAruco(aruco &detector){
    while(true){ //Multiplied by 100 in order to display it in cm.
        std::cout << "forward: " << detector.forward*100 << " right left: " << detector.rightLeft*100 << " updown: " << detector.upDown*100
                  << " angle: " << detector.leftOverAngle.first << " clockwise: " << detector.leftOverAngle.second << std::endl;
                  
        if(detector.ID==-1){
        lostLeader(detector);
        }
    }
}

void distances(drone& drone, aruco& detector) {
    if (detector.inFormation == true) {
        drone.distanceForward = (detector.forward * 100) - FORWARD;
        drone.distanceRightLeft = std::abs(detector.rightLeft * 100) - RIGHT_LEFT;
        drone.distanceHeight = std::abs(detector.upDown) > HEIGHT ? detector.upDown : 0;
        drone.angle = detector.leftOverAngle.first;
        isLost = false;
        updateMovement(drone, detector);
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

int main(){
    std::ifstream programData("../config.json");

    drone d1 ;

    nlohmann::json data;
    programData >> data;
    programData.close();
    std::string yamlCalibrationPath = data["yamlCalibrationPath"];
    bool isCameraString = data["isCameraString"];
    float currentMarkerSize = data["currentMarkerSize"];
    if (isCameraString){
        std::string cameraString = data["cameraString"];
        aruco detector(yamlCalibrationPath,cameraString,currentMarkerSize);
        runAruco(detector);
    }else{
        int cameraPort = data["cameraPort"];
        aruco detector(yamlCalibrationPath,cameraPort,currentMarkerSize);
        runAruco(detector);
    }
}

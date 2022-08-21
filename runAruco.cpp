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
//#include <math>
#include <ctello.h>

#define FORWARD 40
#define RIGHT_LEFT 30 
#define HEIGHT 3 
#define BUFFER_SIZE 10
//#define PI 3.14159
#define RADIUS 50
#define LIM_FORWARD 10
#define LIM_RIGHT_LEFT 5
#define LIM_HEIGHT 5

std::deque<drone> buffer;
bool isLost = false;


// update drones movement with regards to leader.
void updateMovement(drone& drone, aruco& detector, ctello::Tello& tello) {
	

}


void circularAngle(drone &d1, double angle){
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

		drone.distanceHeight = std::abs(detector.upDown) > LIM_HEIGHT ? detector.upDown : 0;

		drone.angle = detector.leftOverAngle.first; // Maybe we shall add angle limit.
		isLost = false;
		circularAngle(drone, detector.leftOverAngle.first);

		//std::cout<<" drone: "<< std::endl;
		//std::cout<<"forward: "<< drone.distanceForward << " rightLeft :"<< drone.distanceRightLeft<< " height: "<<drone.distanceHeight<< " angle: "<< drone.angle  << std::endl;


		updateMovement(drone, detector, tello);
	}

}

//In case of not finding a leader, moving in the direction of the last frame captured.
void lostLeader(aruco &detector, ctello::Tello& tello) {
    int i=0;
    if (!isLost) {
        isLost = true;
	drone lastFrame = buffer.front();  // 
        distances(lastFrame, detector, tello);  // 
        
  
  }
}


void runAruco(aruco &detector, drone &d1, ctello::Tello& tello){
    int tmpId;
    int printFlag=100;
    while(true){ //Multiplied by 100 in order to display it in cm.
    
    
    if(printFlag==100){
      std::cout<< " detector: "<< std::endl; 
      std::cout << "forward: " << detector.forward*100 << " right left: " << detector.rightLeft*100 << " updown: " << detector.upDown*100
                  << " angle: " << detector.leftOverAngle.first << " clockwise: " << detector.leftOverAngle.second <<"  ID: "<< detector.ID <<"  right or left: "<< detector.rightInForm<< " init: " << detector.init << std::endl;
                  printFlag=0;
                  }
                  printFlag++;
       if(detector.ID!=-1){
       		tmpId=detector.ID;
       		}           
                  
        
        int i=0;      
        if(detector.ID==-1){
            while(i<5){// timer for leader search when lost. after 5 seconds of not finding the leader stop searching and stop in place.
                lostLeader(detector, tello);
                //sleep(1); 
            	//std::cout<<"i="<<i<<std::endl;
            	i++;
            	if(detector.ID!=tmpId && detector.ID!=-1)
            		detector.init=true;
            }
            while(detector.ID==-1);
      	    detector.init=true;
        }else{
        //std::cout<<"  detector"<<detector.init<<std::endl;          
        distances(d1,detector, tello);
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

//ctello::~Tello(){ }

int main(){
    std::ifstream programData("../config.json");

    drone d1 ;


    nlohmann::json data;
    programData >> data;
    programData.close();
         std::string droneName = data["DroneName"];
    std::string commandString = "nmcli c up " + droneName;
    const char *command = commandString.c_str();
    system(command);
    ctello::Tello tello;
        tello.SendCommandWithResponse("streamon");
    std::string yamlCalibrationPath = data["yamlCalibrationPath"];
    bool isCameraString = data["isCameraString"];
    float currentMarkerSize = data["currentMarkerSize"];
    
    
    
    if (isCameraString){
        std::string cameraString = data["cameraString"];
        aruco detector(yamlCalibrationPath,cameraString,currentMarkerSize);
        //std::thread arucoThread = std::move(std::thread(&runAruco(detector,d1, tello), this));
        runAruco(detector,d1, tello);        
    }else{
        int cameraPort = data["cameraPort"];
        aruco detector(yamlCalibrationPath,cameraPort,currentMarkerSize);
        //std::thread arucoThread = std::move(std::thread(&runAruco(detector,d1, tello), this));        
        runAruco(detector,d1, tello);
    }
    
    //tello.SendCommandWithResponse("land");
    //arucoThread.join();
    return 0; 
}

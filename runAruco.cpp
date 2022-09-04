//
// Created by tzuk on 6/6/22.
//
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <unistd.h>
#include <deque>
#include "aruco.h"
#include "../include/drone.h"
#include <cmath>
#include <ctello.h>

#define FORWARD 160
#define RIGHT_LEFT 20
#define LIM_FORWARD 20
#define LIM_RIGHT_LEFT 20
#define LIM_HEIGHT 15
#define LIM_ANGLE 20
//#define LIM_ANGLE_CIRCLE 25
#define LIM_MOVEMENT 30
#define LIM_MOVEMENT_HEIGHT 10
#define LIM_MOVEMENT_ANGLE 25

const std::string noMovement = "0 ";	


// update drones movement with regards to leader.
void updateMovement(drone& drone, aruco& detector, ctello::Tello& tello) {
	
	int tmpId=-1;
	
	while(true) {
	       while(!drone.commandFlag && detector.ID!=-1);
		
	       if(detector.ID!=-1){
       			tmpId=detector.ID;
       		}
       		  
		if(detector.ID==-1 && tmpId!=-1){
		    int i=0;
		    while(detector.ID==-1 && i<50){
			    sleep(2); 
			    std::cout << "Searching for leader" << std::endl;
			    tello.SendCommand("rc 0 0 0 25");
		    	    i++;
    		   	    if(detector.ID!=tmpId && detector.ID!=-1)
    			          detector.init=true;
		    }
		    
		    if(detector.ID==-1)
		    	tello.SendCommandWithResponse("land");
      	    			
        }else{
		std::string command = "rc ";
		
		if (drone.distanceRightLeft){
			if(std::abs(drone.distanceRightLeft*0.4) > LIM_MOVEMENT)
				drone.distanceRightLeft = drone.distanceRightLeft > 0 ? LIM_MOVEMENT : -1 * LIM_MOVEMENT;
			command += std::to_string((int)std::trunc(drone.distanceRightLeft*0.4));	
			command+=" ";
		}
		else
			command += noMovement;

		if (drone.distanceForward){
		
			if(std::abs(drone.distanceForward*0.4) > LIM_MOVEMENT)
				drone.distanceForward = drone.distanceForward > 0 ? LIM_MOVEMENT : -1 * LIM_MOVEMENT;
				
			command +=std::to_string((int)std::trunc(drone.distanceForward*0.4));
			command+=" ";
		}
		else
			command += noMovement;


		if (drone.distanceHeight){
		
			if(std::abs(drone.distanceHeight) > LIM_MOVEMENT_HEIGHT)
				drone.distanceHeight = drone.distanceHeight > 0 ? LIM_MOVEMENT_HEIGHT : -1 * LIM_MOVEMENT_HEIGHT;
		
			command += std::to_string((int)std::trunc(drone.distanceHeight));
			command+=" ";
		}
		else
			command += noMovement;

		if (drone.angle){
		
			if(std::abs(drone.angle) > LIM_MOVEMENT_ANGLE)
				drone.angle = drone.angle > 0 ? LIM_MOVEMENT_ANGLE : -1 * LIM_MOVEMENT_ANGLE;
		
			command += std::to_string((int)std::trunc(drone.angle));
			command+=" ";
		}
		else
			command += noMovement;
		
                  
		try {
				
			if(!detector.init || detector.ID!=-1){
				tello.SendCommand(command);	
			}
			else 
				tello.SendCommand("rc 0 0 0 0");
			
			usleep(500000);
			
			
		std::cout << command << std::endl;
		
		std::cout << "drone:" <<" height: "<< drone.distanceHeight << " forward: "<< drone.distanceForward << " rightLeft: " << drone.distanceRightLeft << " angle: "<< drone.angle << std::endl;
			
		std::cout << "detector:" << " position" << (detector.rightInForm > 0 ? " RIGHT" : " LEFT") << " forward: " << detector.forward << " right-left: " << detector.rightLeft << " updown: " << detector.upDown	<< " angle: " <<  detector.yaw << " roll: "<< detector.rollAngle << " init: " << detector.init << " ID " << detector.ID   << std::endl;
                 
                 
                std::cout << std::endl;
                 
		}catch(...){
		
		}
		
	}
		drone.commandFlag = false;
	}
}


void circularAngle(drone &d1, double angle, aruco  &detector){
	double radius = std::sqrt(std::pow(detector.forward,2) + std::pow(detector.rightLeft,2));
	d1.distanceForward += (radius - (radius * std::cos(angle * M_PI / 180)));
	d1.distanceRightLeft += (radius * std::sin(angle * M_PI / 180)) * -0.25;
}



void distances(drone& drone, aruco& detector, ctello::Tello& tello) {

	if (!detector.init) {
		if (std::abs(detector.forward - FORWARD) > LIM_FORWARD)
			drone.distanceForward = detector.forward - FORWARD + 0.5*LIM_FORWARD;
		else
			drone.distanceForward = 0;


		
		if(std::abs(std::abs(detector.rightLeft - detector.yaw) - RIGHT_LEFT) > LIM_RIGHT_LEFT) {
			drone.distanceRightLeft = -(detector.rightLeft - detector.yaw) + detector.rightInForm *( RIGHT_LEFT + LIM_RIGHT_LEFT * 0.5 );
		} else{  
			drone.distanceRightLeft = 0;
		}
	
		if(detector.rollAngle > 0){
			drone.distanceHeight = detector.upDown + (180 - detector.rollAngle);
		} else {
			drone.distanceHeight = detector.upDown - (180 + detector.rollAngle);
		}

		
		if(std::abs(drone.distanceHeight) > LIM_HEIGHT){
			if(detector.upDown > 0)
				drone.distanceHeight = -1 * (drone.distanceHeight - LIM_HEIGHT * 0.5);
			else
				drone.distanceHeight = -1 * (drone.distanceHeight + LIM_HEIGHT * 0.5);
		} else 
			drone.distanceHeight = 0;
		
		if (std::abs(detector.yaw) > LIM_ANGLE){
			drone.angle = detector.yaw > 0 ? (detector.yaw - (LIM_ANGLE*0.5)) : (detector.yaw + (LIM_ANGLE*0.5));
			//if(std::abs(detector.yaw) > LIM_ANGLE_CIRCLE)
			circularAngle(drone, drone.angle, detector);
		}else
			drone.angle = 0;


		drone.commandFlag = true;
	}

}


void runAruco(aruco &detector, drone &d1, ctello::Tello& tello){
    while(true){
   
        if(detector.ID!=-1){        
        	distances(d1,detector, tello);
        }
    }
}



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
    
    tello.SendCommandWithResponse("takeoff");  

				
    
    if (isCameraString){
        std::string cameraString = data["cameraString"];
        aruco detector(yamlCalibrationPath,cameraString,currentMarkerSize);
 	std::thread movementThread([&] { updateMovement(d1, detector,tello); } );        
        runAruco(detector,d1, tello);       
        movementThread.join(); 
    }else{
        int cameraPort = data["cameraPort"];
        aruco detector(yamlCalibrationPath,cameraPort,currentMarkerSize);
	std::thread movementThread([&] { updateMovement(d1, detector,tello); } );        
        runAruco(detector,d1, tello);
        movementThread.join();   
    }
    
    return 0; 
}

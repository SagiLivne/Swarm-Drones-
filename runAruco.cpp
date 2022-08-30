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

#define FORWARD 120
#define LEFT 30
#define RIGHT 15
#define BUFFER_SIZE 10	
#define RADIUS 0
#define LIM_FORWARD 20
#define LIM_RIGHT_LEFT 20
#define LIM_HEIGHT 8
#define LIM_ANGLE 15
#define LIM_ANGLE_CIRCLE 20

std::deque<drone> buffer;
bool isLost = false;

const std::string verticalMovement = "";
const std::string noMovement = "0 ";
const std::string upMovement = "5 ";
const std::string horizontalMovement = "";



// update drones movement with regards to leader.
void updateMovement(drone& drone, aruco& detector, ctello::Tello& tello) {
	
	int tmpId=-1;
	
	while(true) {
		//while(!drone.commandFlag);
			//std::cout << "got into the busy wait" << std::endl;
		

	       if(detector.ID!=-1 ){
       		tmpId=detector.ID;
       		}  
		if(detector.ID==-1 && tmpId!=-1){
		int i=0;
		drone.distanceHeight = drone.distanceHeight /10; 
		    while(i<10){// timer for leader search when lost. after 5 seconds of not finding the leader stop searching and stop in place.
		        //lostLeader(detector, tello);
		        sleep(0.25); 
		    	i++;

		    }

		    i=0;
		    while(detector.ID==-1 && i<20){
			    sleep(2); 
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
			if(std::abs(drone.distanceRightLeft*0.4) > 40)
				drone.distanceRightLeft = drone.distanceRightLeft > 0 ? 40 : -40;
			command += std::to_string(drone.distanceRightLeft*0.4);	
			command+=" ";
		}
		else
			command += noMovement;

		if (drone.distanceForward){
		
			if(std::abs(drone.distanceForward*0.4) > 40)
				drone.distanceForward = drone.distanceForward > 0 ? 40 : -40;
				
			command +=std::to_string(drone.distanceForward*0.4);
			command+=" ";
		}
		else
			command += noMovement;


		if (drone.distanceHeight){
		
			if(std::abs(drone.distanceHeight) > 40)
				drone.distanceHeight = drone.distanceHeight > 0 ? 40 : -40;
		
			command += std::to_string(drone.distanceHeight);
			command+=" ";
		}
		else
			command += noMovement;

		if (drone.angle){
		
			if(std::abs(drone.angle) > 25)
				drone.angle = drone.angle > 0 ? 25 : -25;
		
			command += std::to_string(drone.angle);
			command+=" ";
		}
		else
			command += noMovement;
		
                  
		try {
				
			if(!detector.init || detector.ID!=-1)
				tello.SendCommand(command);	
			else 
				tello.SendCommand("rc 0 0 0 0");
			
			usleep(50);
			std::cout /*<< command */<< "drone:" /*<<" height: "<< drone.distanceHeight << "forward: "<< drone.distanceForward*/ << " rightLeft :" << drone.distanceRightLeft << " angle: "<< drone.angle /*<< "Right or Left " << detector.rightInForm *//*<< " forward: " << detector.forward*/ <<" right-left: " << detector.rightLeft /*<< " updown: " << detector.upDown*/	
                  << " angle: " <<  detector.yaw/* <<"  ID: "<< detector.ID <<"  right or left: "<< detector.rightInForm *//*<< "Angle: " << (detector.leftOverAngle.second ? detector.leftOverAngle.first : (-detector.leftOverAngle.first))/*<< " init: " << detector.init /*<< " Roll? :" << detector.rollAngle <<" updown: " << detector.upDown */<< std::endl;
                 
		}catch(...){
		
		}
		
	}
		drone.commandFlag = false;
	}
}


void circularAngle(drone &d1, double angle, aruco  &detector){
	double radius = std::sqrt(std::pow(detector.forward,2) + std::pow(detector.rightLeft,2)) - RADIUS;
	d1.distanceForward += (radius - (radius * std::cos(angle * M_PI / 180)));
	d1.distanceRightLeft += (radius * std::sin(angle * M_PI / 180)) * -0.25;
}


void distances(drone& drone, aruco& detector, ctello::Tello& tello) {

	if (!detector.init) {
		//while(drone.commandFlag);
		if (std::abs(detector.forward - FORWARD) > LIM_FORWARD)
			drone.distanceForward = detector.forward - FORWARD + 0.5*LIM_FORWARD;
		else
			drone.distanceForward = 0;


		if(detector.rightInForm > 0){
			if(std::abs(detector.rightLeft - detector.yaw - RIGHT) > LIM_RIGHT_LEFT) {
				 
				drone.distanceRightLeft = -(detector.rightLeft - detector.yaw) + detector.rightInForm *( RIGHT + LIM_RIGHT_LEFT * 0.5 );
			} else{  
				drone.distanceRightLeft = 0;
			}
		}
		else{
			if(std::abs(detector.rightLeft - detector.yaw - LEFT) > LIM_RIGHT_LEFT) {
				drone.distanceRightLeft = -(detector.rightLeft - detector.yaw) + detector.rightInForm *( LEFT + LIM_RIGHT_LEFT * 0.5 );
			} else{ 
				drone.distanceRightLeft = 0;
			}
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
			if(std::abs(detector.yaw) > LIM_ANGLE_CIRCLE)
				circularAngle(drone, drone.angle, detector);
		}else
			drone.angle = 0;


		drone.commandFlag = true;
		isLost = false;
	}

}

//In case of not finding a leader, moving in the direction of the last frame captured.
/*void lostLeader(aruco &detector, ctello::Tello& tello) {
    int i=0;
    if (!isLost) {
        isLost = true;
	//drone lastFrame = buffer.front();  
        //distances(lastFrame, detector, tello);          
  
  }
}*/


void runAruco(aruco &detector, drone &d1, ctello::Tello& tello){
    while(true){ //Multiplied by 100 in order to display it in cm.
   
        if(detector.ID!=-1){        
        	distances(d1,detector, tello);
        }
    }
}




//Adding new drone frame to the buffer.
/*void addToBuffer(drone& dr) {
    if (buffer.size() == BUFFER_SIZE) {
        drone d1 = buffer.back();
        buffer.pop_back();
    }
    buffer.push_front(dr);
}
*/


// insert a drone into and existing formation
void insertToFormation() {

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
    
    //tello.SendCommandWithResponse("land");
    return 0; 
}

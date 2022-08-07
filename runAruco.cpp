//
// Created by tzuk on 6/6/22.
//
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <unistd.h>
#include "aruco.h"
void runAruco(aruco &detector){
    while(true){ //Multiplied by 100 in order to display it in cm.
        std::cout << "forward: " << detector.forward*100 << " right left: " << detector.rightLeft*100 << " updown: " << detector.upDown*100
                  << " angle: " << detector.leftOverAngle.first << " clockwise: " << detector.leftOverAngle.second << std::endl;
    }
}
int main(){
    std::ifstream programData("../config.json");
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

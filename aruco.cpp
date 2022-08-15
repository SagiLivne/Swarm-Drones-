//
// Created by tzuk on 6/6/22.
//

#include <unistd.h>
#include "../include/aruco.h" //Might not need
#include "../Drone.h"

std::vector<cv::Mat> aruco::getCameraCalibration(const std::string &path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) throw std::runtime_error("CameraParameters::readFromXMLFile could not open file:" + path);
    int w = -1, h = -1;
    cv::Mat MCamera, MDist;

    fs["image_width"] >> w;
    fs["image_height"] >> h;
    fs["distortion_coefficients"] >> MDist;
    fs["camera_matrix"] >> MCamera;

    if (MCamera.cols == 0 || MCamera.rows == 0) {
        fs["Camera_Matrix"] >> MCamera;
        if (MCamera.cols == 0 || MCamera.rows == 0)
            throw cv::Exception(9007, "File :" + path + " does not contains valid camera matrix",
                                "CameraParameters::readFromXML", __FILE__, __LINE__);
    }

    if (w == -1 || h == 0) {
        fs["image_Width"] >> w;
        fs["image_Height"] >> h;
        if (w == -1 || h == 0)
            throw cv::Exception(9007, "File :" + path + " does not contains valid camera dimensions",
                                "CameraParameters::readFromXML", __FILE__, __LINE__);
    }
    if (MCamera.type() != CV_32FC1)
        MCamera.convertTo(MCamera, CV_32FC1);

    if (MDist.total() < 4) {
        fs["Distortion_Coefficients"] >> MDist;
        if (MDist.total() < 4)
            throw cv::Exception(9007, "File :" + path + " does not contains valid distortion_coefficients",
                                "CameraParameters::readFromXML", __FILE__, __LINE__);
    }
    MDist.convertTo(MDist, CV_32FC1);
    std::vector<cv::Mat> newCameraParams = {MCamera, MDist};
    return newCameraParams;
}

void aruco::trackMarkerThread() {
    stop = false;
    std::cout << "started track thread" << std::endl;
    std::vector<std::vector<cv::Point2f>> corners;
    const std::vector<cv::Mat> cameraParams = getCameraCalibration(yamlCalibrationPath);
    const cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(
            cv::aruco::DICT_ARUCO_ORIGINAL);
    while (!stop) {
        std::vector<int> ids;
        if (frame && !frame->empty()) {
            cv::aruco::detectMarkers(*frame, dictionary, corners, ids);
        } else {
            std::cout << "no frames" << std::endl;
            lostLeader();
            ID = -1;
            sleep(1);
            continue;
        }
        bool canContinue = true;
        int rightId = 0;
        /*while (rightId < ids.size()) {
            if (ids[rightId] != 0) {
                canContinue = true;
                break;
            }
            rightId++;
        }*/
        // if at least one marker detected
        if (canContinue) {
            std::vector<cv::Vec3d> localRvecs, localTvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, currentMarkerSize, cameraParams[0], cameraParams[1],
                localRvecs,
                localTvecs);

            if (init) {
                initialaize(twoClosest(localRvecs, localTvecs);
                init = false;
            }else{
            rightID = twoClosest(localRvecs, localTvecs).first;

            if (!localRvecs.empty()) {
                cv::Mat rmat = cv::Mat::eye(3, 3, CV_64FC1);
                try {
                    cv::Rodrigues(localRvecs[rightId], rmat);
                }
                catch (...) {
                    std::cout << "coudlnot convert vector to mat" << std::endl;
                    continue;
                }
                auto t = cv::Mat(-rmat.t() * cv::Mat(localTvecs[rightId]));
                rightLeft = t.at<double>(0);
                upDown = t.at<double>(1);
                forward = t.at<double>(2);
                ID = rightID;
                leftOverAngle = getLeftOverAngleFromRotationVector(localRvecs[rightId]);
                usleep(amountOfUSleepForTrackMarker);
            }
            else {
                std::cout << "couldnt get R vector" << std::endl;
            }
            }
        } else {
            std::cout << "didnt detect marker" << std::endl;
        }
        // usleep(100000);
    }
}

//initialize drones
void initialaize(std::pair<int, int> twoClosest, std::vector<cv::Vec3d> localTvecs;) {
    cv::Mat rmat = cv::Mat::eye(3, 3, CV_64FC1);

    try {
        cv::Rodrigues(localRvecs[rightId], rmat);
    }
    catch (...) {
        std::cout << "coudlnot convert vector to mat" << std::endl;
        continue;
    }

    auto t = cv::Mat(-rmat.t() * cv::Mat(localTvecs[twoClosest.first]));
    double rightLeftInit = t.at<double>(0);

    auto t = cv::Mat(-rmat.t() * cv::Mat(localTvecs[twoClosest.second]));
    double rightLeftInit2 = t.at<double>(0);
     
    if (rightleftInit < rightLeftInit2) {
        rightInForm = false;
    }
    else {
        rightInForm = true;
    }

}

//Returns the two closest markers detected.
std::pair<int, int> twoClosest(std::vector<cv::Vec3d> localRvecs, std::vector<cv::Vec3d> localTvecs) {
    double firstMinForward;
    double secondMinForward;
    int firstMinID = 0, secondMinID = 0;
    if (!localRvecs.empty()) {
        if (localRvecs.size == 1) {
            return std::make_pair<int, int>(0,0);
        }
        firstMinForward = forwardDistance(localRvecs[0], localTvecs[0]);
        secondMinForward = forwardDistance(localRvecs[1],localTvecs[1]);
        if (firstMinForward > secondMinForward) {
            firstMinForward= forwardDistance(localRvecs[1], localTvecs[1]);
            secondMinforward = forwardDistance(localRvecs[0], localTvecs[0]);
        }
        for (int iter = 2; iter < localRvecs.size; iter++) {
            double iterForward = forwardDistance(localRvecs[iter], localTvecs[iter]);
            if (firstMinForward > iterForward) {
                secondMinForward = firstMinForward; 
                secondMinID = firstMinID;
                firstMinForward = iterForward;
                firstMinID = iter;
            }
            else {
                if (secondMinForward > iterForward) {
                    secondMinForward = iterForward;
                    secondMinID = iter;
                }
            }
        }
    } else {
        std::cout << "couldnt get R vector" << std::endl;
    }
    return std::make_pair<int, int> (firstMinID, secondMinID);
}

//Try to generalize to return t var instead.
double forwardDistance(std::vector<cv::Vec2d> localRvec, std::vector<cv::Vec2d> localTvec) {
    cv::Mat rmat = cv::Mat::eye(3, 3, CV_64FC1);
    try {
        cv::Rodrigues(localRvec, rmat);
    }
    catch (...) {
        std::cout << "coudlnot convert vector to mat" << std::endl;
        continue;
    }
    auto t = cv::Mat(-rmat.t() * cv::Mat(localTvec));
    return t.at<double>(2);
}

void aruco::getEulerAngles(cv::Mat &rotCameraMatrix, cv::Vec3d &eulerAngles) {
    cv::Mat cameraMatrix, rotMatrix, transVect, rotMatrixX, rotMatrixY, rotMatrixZ;
    auto *_r = rotCameraMatrix.ptr<double>();
    double projMatrix[12] = {_r[0], _r[1], _r[2], 0,
                             _r[3], _r[4], _r[5], 0,
                             _r[6], _r[7], _r[8], 0};
    decomposeProjectionMatrix(cv::Mat(3, 4, CV_64FC1, projMatrix),
                              cameraMatrix,
                              rotMatrix,
                              transVect,
                              rotMatrixX,
                              rotMatrixY,
                              rotMatrixZ,
                              eulerAngles);
}

std::pair<int, bool> aruco::getLeftOverAngleFromRotationVector(const cv::Vec<double, 3> &rvec) {
    cv::Mat R33 = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Rodrigues(rvec, R33);
    cv::Vec3d eulerAngles;
    getEulerAngles(R33, eulerAngles);
    int yaw = std::abs(int(eulerAngles[1]));
    return {yaw, eulerAngles[1] > 0};
}

void aruco::getCameraFeed() {
    runCamera = true;
    while (runCamera) {
        if (!capture || !(capture->isOpened()) || *holdCamera) {
            usleep(5000);
            continue;
        }
        capture->read(*frame);
        cv::imshow("aruco", *frame);
        cv::waitKey(1);
    }
}

aruco::aruco(std::string &yamlCalibrationPath, int cameraPort, float currentMarkerSize) {
    this->yamlCalibrationPath = yamlCalibrationPath;
    stop = false;
    holdCamera = std::make_shared<bool>(false);
    frame = std::make_shared<cv::Mat>();
    capture = std::make_shared<cv::VideoCapture>();
    if (capture->open(cameraPort)) {
        std::cout << "camera opened" << std::endl;
    } else {
        std::cout << "couldnt open camera by port" << std::endl;
    }
    this->currentMarkerSize = currentMarkerSize;
    cameraThread = std::move(std::thread(&aruco::getCameraFeed, this));
    arucoThread = std::move(std::thread(&aruco::trackMarkerThread, this));
}

aruco::aruco(std::string &yamlCalibrationPath, std::string &cameraString, float currentMarkerSize) {
    this->yamlCalibrationPath = yamlCalibrationPath;
    stop = false;
    holdCamera = std::make_shared<bool>(false);
    frame = std::make_shared<cv::Mat>();
    capture = std::make_shared<cv::VideoCapture>();
    capture->open(cameraString, cv::CAP_FFMPEG);
    this->currentMarkerSize = currentMarkerSize;
    cameraThread = std::move(std::thread(&aruco::getCameraFeed, this));
    arucoThread = std::move(std::thread(&aruco::trackMarkerThread, this));

}

aruco::aruco(float)


aruco::~aruco() {
    stop = true;
    runCamera = false;
    cameraThread.join();
    arucoThread.join();
    capture->release();
}

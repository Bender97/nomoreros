//
// Created by fusy on 13/12/21.
//

#ifndef PROJECTING_IMULOADER_H
#define PROJECTING_IMULOADER_H

#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>
#include <ctime>

class IMULoader {
public:
    double lat, lon, alt;
    double roll, pitch, yaw;

    double vn, ve, vf, vl, vu;
    double ax, ay, az, af, al, au;
    double wx, wy, wz, wf, wl, wu;

    cv::Vec4f  orientation;
    double orientation_covariance[9];

    cv::Vec3f angular_velocity;
    double angular_velocity_covariance[9];

    cv::Vec3f linear_acceleration;
    double linear_acceleration_covariance[9];

    int64 timestamp;
    int64 nanosecs;

    IMULoader(std::string path);
    void loadIMU_from_file(std::string path);
    void load_time();

};

#endif //PROJECTING_IMULOADER_H

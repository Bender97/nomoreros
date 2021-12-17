//
// Created by fusy on 13/12/21.
//

#include "IMULoader.h"

IMULoader::IMULoader() {
    orientation_covariance[0] = 1.0;
    orientation_covariance[1] = .0;
    orientation_covariance[2] = .0;
    orientation_covariance[3] = .0;
    orientation_covariance[4] = 1.0;
    orientation_covariance[5] = .0;
    orientation_covariance[6] = .0;
    orientation_covariance[7] = .0;
    orientation_covariance[8] = 1.0;

    angular_velocity_covariance[0] = 1.2184696791468346e-07;
    angular_velocity_covariance[1] = .0;
    angular_velocity_covariance[2] = .0;
    angular_velocity_covariance[3] = .0;
    angular_velocity_covariance[4] = 1.2184696791468346e-07;
    angular_velocity_covariance[5] = .0;
    angular_velocity_covariance[6] = .0;
    angular_velocity_covariance[7] = .0;
    angular_velocity_covariance[8] = 1.2184696791468346e-07;

    linear_acceleration_covariance[0] = 8.999999999999999e-08;
    linear_acceleration_covariance[1] = .0;
    linear_acceleration_covariance[2] = .0;
    linear_acceleration_covariance[3] = .0;
    linear_acceleration_covariance[4] = 8.999999999999999e-08;
    linear_acceleration_covariance[5] = .0;
    linear_acceleration_covariance[6] = .0;
    linear_acceleration_covariance[7] = .0;
    linear_acceleration_covariance[8] = 8.999999999999999e-08;
}

void IMULoader::loadIMU_from_file() {
    std::string path = "/home/fusy/bags/2011_09_26_drive_0002_sync/2011_09_26/2011_09_26_drive_0002_sync/oxts/data/0000000000.txt";
    std::string line;
    std::ifstream in(path);

    in >> lat;
    in >> lon;
    in >> alt;

    in >> roll;
    in >> pitch;
    in >> yaw;

    in >> vn; in >> ve; in >> vf; in >> vl; in >> vu;
    in >> ax; in >> ay; in >> az; in >> af; in >> al; in >> au;
    in >> wx; in >> wy; in >> wz; in >> wf; in >> wl; in >> wu;

    in.close();

    std::cout << "wx: " << wz << std::endl;
    std::cout << "wy: " << wy << std::endl;
    std::cout << "wz: " << wz << std::endl;

    load_time();

}

time_t convertTimeToEpoch(const char* theTime, const char* format = "%Y-%m-%d %H:%M:%S")
{
    std::tm tmTime;
    memset(&tmTime, 0, sizeof(tmTime));
    strptime(theTime, format, &tmTime);
    return mktime(&tmTime);
}

void IMULoader::load_time() {
    std::string path = "/home/fusy/bags/2011_09_26_drive_0002_sync/2011_09_26/2011_09_26_drive_0002_sync/oxts/timestamps.txt";
    std::string date;
    std::string time_nano;

    std::ifstream in(path);
    in >> date;
    in >> time_nano;
    in.close();

    std::string theTime = date + " " + time_nano.substr(0, 8);

    std::tm tmTime;
    memset(&tmTime, 0, sizeof(tmTime));
    strptime(theTime.c_str(), "%Y-%m-%d %H:%M:%S", &tmTime);
    timestamp = mktime(&tmTime);
}
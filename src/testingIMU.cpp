#include "IMULoader.h"
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "IMULoader.h"

//#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <data.h>

const Data data;
/*
//// linear velocity at time zero
// float vx = 6.98390467761731f;
// float vy = 0.18333130383767f;
// float vz = 0.15356837236718f;
//
//// linear acceleration
// float ax = 1.18885125806239f;
// float ay = 0.75031638425215f;
// float az = 13.01972102311150f;
//
//// angular velocity
// float wx = 0.17625848913349f;
// float wy = 0.19206552946943f;
// float wz = 0.02318471760338f;*/


 double imutime     = .335092332;  //actually not used, just needed to know wheter imu data is between velo time limits
 double velo_start  = .265816674;
 double velo_end    = .369008876;
 double veloperiodhalf  = (velo_end - velo_start)/2.0;
 double imgtime     = .327963904;

// linear velocity
 float vx = 8.6225009735591f;
 float vy = -0.023377717262865f;
 float vz = -0.14317006071049f;
// linear acceleration
 float ax = -0.041490794905849f;
 float ay = -0.20837820539826f;
 float az = 9.093745089874f;
// angular velocity
 float wx = 0.023343899978178f;
 float wy = -0.070017406558051f;
 float wz = -0.049960205878114f;

 /*
//// linear velocity at time zero
// float vx = 8.6713784653144f;
// float vy = -0.019461233360934f;
// float vz = -0.095766948135752f;
//
//// linear acceleration
// float ax = 0.36472797260825f;
// float ay = -0.60402693871872f;
// float az = 9.093745089874f;
//
//// angular velocity
// float wx = -0.0098349511973597f;
// float wy = -0.04121262160891f;
// float wz = -0.041177780031438f;*/

Eigen::Transform<float, 3, 2> transStartInverse;

bool show(cv::Mat &img, int timeout) {
    cv::Mat copy;
    cv::resize(img, copy, cv::Size(1000, 1000));
    cv::imshow("prova", copy);
    if (cv::waitKey(timeout) == 'q') return false;
    return true;
}

void loadCloud(std::string lidarpath, std::vector<pcl::PointXYZI> &cloud) {
    float f;
    std::ifstream fin(lidarpath, std::ios::binary);
    assert(fin.is_open());

    pcl::PointXYZI p;
    int cont = 0;

    cloud.clear();

    while (fin.read(reinterpret_cast<char*>(&f), sizeof(float))) {
        switch(cont) {
            case 0:
                p.x = f;
                break;
            case 1:
                p.y = f;
                break;
            case 2:
                p.z = f;
                break;
            case 3:
                cloud.push_back(p);
                break;
        }
        cont = (cont+1)%4;
    }
    fin.close();
}

cv::Point2f projectPoint(double x, double y, double z) {
    double xt, yt, zt; // the 3D velodyne point projected to the 3D camera frame
    double u, v, w; // the projection of the 3D point to the 2D camera frame (not scaled)
    // the (scaled) projection of the 3D point to the 2D camera frame

    // project the point to the 3D camera frame using the transformation matrix of the camera
    xt = data.Tr00*x + data.Tr01*y + data.Tr02*z + data.Tr03;
    yt = data.Tr10*x + data.Tr11*y + data.Tr12*z + data.Tr13;
    zt = data.Tr20*x + data.Tr21*y + data.Tr22*z + data.Tr23;

    // project the point to the 2D camera frame using the camera projection matrix
    u = data.P200*xt + data.P201*yt + data.P202*zt + data.P203;
    v = data.P210*xt + data.P211*yt + data.P212*zt + data.P213;
    w = data.P220*xt + data.P221*yt + data.P222*zt + data.P223;

    // scale the pixel
    cv::Point2f scaled;
    scaled.x = std::roundf(u / w);
    scaled.y = std::roundf(v / w);
    return scaled;
}

double minyaw = 10000.0;
double maxyaw = -10000.0;

double mindt = 10000.0;
double maxdt = -10000.0;

#define GETYAW(x, y) ( atan2((y), (x)) + (M_PI)/4 )

void deskewPoint(pcl::PointXYZI &p) {
    double yaw = GETYAW(p.y, p.x);
    double dt = (velo_start + veloperiodhalf * yaw / (M_PI_2)) - imgtime;

    double dx = -.5*ax*dt*dt + vx*dt;
    double dy = -.5*ay*dt*dt + vy*dt;
    double dz = -.5*az*dt*dt + vz*dt;

    double orient_x = wx*dt;
    double orient_y = wy*dt;
    double orient_z = wz*dt;

    auto Trans = pcl::getTransformation(dx, dy, dz, orient_x, orient_y, orient_z);
//    auto Trans = transStartInverse*trans_final;

    double x = Trans(0,0) * p.x + Trans(0,1) * p.y + Trans(0,2) * p.z + Trans(0,3);
    double y = Trans(1,0) * p.x + Trans(1,1) * p.y + Trans(1,2) * p.z + Trans(1,3);
    double z = Trans(2,0) * p.x + Trans(2,1) * p.y + Trans(2,2) * p.z + Trans(2,3);

    p.x = x;
    p.y = y;
    p.z = z;

}

void projectToImage(std::vector<pcl::PointXYZI> &points, cv::Mat &img) {

    cv::Mat copy = img.clone();

    for (size_t i=0; i<points.size(); i++) {
        pcl::PointXYZI p = points[i];

//        deskewPoint(p);

        if (p.x<0) continue;

        cv::Point2f scaled = projectPoint(p.x, p.y, p.z);
        if (scaled.x >= 0 && scaled.x<img.cols && scaled.y>=0 && scaled.y<img.rows) {
            auto & color = copy.at<cv::Vec3b>((int)scaled.y, (int)scaled.x);
            color[0] = 0;
            color[1] = 0;
//            double dist = std::sqrt(p.x*p.x + p.y*p.y);
//            std::cout << dist << std::endl;
//            color[2] = 255-MIN(255, (int)(dist*255.0/20.0));
            color[2] = 255;
        }
    }
    cv::Mat resized;
    cv::resize(copy, resized, cv::Size(img.cols*2, img.rows*2));

    cv::imshow("wind", resized);
    cv::waitKey(0);
}
void projectToImage1(std::vector<pcl::PointXYZI> &points, cv::Mat &img) {
    cv::Mat copy = img.clone();

    for (size_t i=0; i<points.size(); i++) {
        pcl::PointXYZI p = points[i];

//        deskewPoint(p);

        if (p.x<0) continue;
        cv::Point2f scaled = projectPoint(p.x, p.y, p.z);
        if (scaled.x >= 0 && scaled.x<img.cols && scaled.y>=0 && scaled.y<img.rows) {
            auto & color = copy.at<cv::Vec3b>((int)scaled.y, (int)scaled.x);
            color[0] = 0;
            color[1] = 0;
            color[2] = 255;
        }
    }
    cv::Mat resized;
    cv::resize(copy, resized, cv::Size(img.cols*2, img.rows*2));

    cv::imshow("wind", resized);
    cv::waitKey(0);
}

void drawPoint(cv::Mat &canvas, pcl::PointXYZI &p, int b, int g, int r) {
    int px = canvas.cols/2 + p.y*10;
    int py = canvas.rows/2 + p.x*10;
    if (px>=0 && px<canvas.cols && py>=0 && py<canvas.rows) {
        auto &pix = canvas.at<cv::Vec3b>(py, px);
        pix[0] = b;
        pix[1] = g;
        pix[2] = r;
    }
}
void drawPoint2(cv::Mat &canvas, pcl::PointXYZI &p, int b, int g, int r) {
    int px = canvas.cols/2 + p.y*10;
    int py = canvas.rows/2 + p.x*10;
    if (px>=0 && px<canvas.cols && py>=0 && py<canvas.rows) {
        cv::Rect rect(py-10, px-10, 20, 20);
        cv::rectangle(canvas, rect, cv::Scalar(b, g, r), 3);

        cv::line(canvas, cv::Point2f(canvas.rows/2, canvas.cols/2), cv::Point2f(py, px), cv::Scalar(b, g, r), 3);

    }
    else {
        std::cout << "POINT OOS" << std::endl;
    }
}


void projectToImage2(std::vector<pcl::PointXYZI> &points, cv::Mat &img) {
    std::vector<float> dists(points.size());
    for (size_t i=0; i<points.size(); i++) {
        if (points[i].x < 0) continue;

        cv::Point2f scaled = projectPoint(points[i].x, points[i].y, points[i].z);
        if (scaled.x >= 0 && scaled.x < img.cols && scaled.y >= 0 && scaled.y < img.rows) {
            double yaw = GETYAW(points[i].x, points[i].y);
            double dt = (velo_start + veloperiodhalf * yaw / (M_PI_2)) - imgtime;
            if (yaw < minyaw) minyaw = yaw;
            else if (yaw > maxyaw) maxyaw = yaw;
            if (dt < mindt) mindt = dt;
            else if (dt > maxdt) maxdt = dt;
        }
    }
    std::cout << "minyaw: " << minyaw << " \t maxyaw: " << maxyaw << std::endl;
    std::cout << "mindt : " << mindt  << " \t maxdt : " << maxdt  << std::endl;

    for (size_t i=0; i<points.size(); i++) {
        deskewPoint(points[i]);
        dists[i] = points[i].x*points[i].x + points[i].y*points[i].y + points[i].z*points[i].z;
    }
    for (float dist = 20.0f; dist>=5.0f; dist-=0.5f) {
        float sqdist = dist*dist;
        cv::Mat copy = img.clone();

        for (size_t i = 0; i < points.size(); i++) {
            pcl::PointXYZI p = points[i];
            if (p.x < 0 || dists[i] > sqdist) continue;

            cv::Point2f scaled = projectPoint(p.x, p.y, p.z);
            if (scaled.x >= 0 && scaled.x < img.cols && scaled.y >= 0 && scaled.y < img.rows) {
                auto &color = copy.at<cv::Vec3b>((int) scaled.y, (int) scaled.x);
                color[0] = 0;
                color[1] = 0;
                color[2] = (int) ( GETYAW(p.x, p.y) * 255.0 / (M_PI_2));
            }
        }
        std::cout << "DIST: " << dist << std::endl;
        cv::Mat resized;
        cv::resize(copy, resized, cv::Size(img.cols * 2, img.rows * 2));
        cv::imshow("wind", resized);
        char c = cv::waitKey(0);
        if (c == 'q') break;
        if (c == 'p') dist += 1.0f;
    }
}

int main() {

    for (int i=0; i<1; i++) {
        std::string old_str = std::to_string(i);
//        std::string img_path = "/home/fusy/bags/00/image_02/data/" + std::string(10 - MIN(10, old_str.length()), '0') + old_str  +".png";
//        std::string lidar_path = "/home/fusy/bags/00/velodyne/" + std::string(6 - MIN(6, old_str.length()), '0') + old_str  + ".bin";
        std::string img_path = "/home/fusy/bags/2011_09_26_drive_0002_sync/2011_09_26/2011_09_26_drive_0002_sync/image_02/data/" + std::string(10 - MIN(10, old_str.length()), '0') + old_str  +".png";
        std::string lidar_path = "/home/fusy/bags/2011_09_26_drive_0002_sync/2011_09_26/2011_09_26_drive_0002_sync/velodyne_points/data/" + std::string(10 - MIN(10, old_str.length()), '0') + old_str  + ".bin";

        std::cout << img_path << std::endl << lidar_path << std::endl;

        std::vector<pcl::PointXYZI> cloud;
//    IMULoader imu;

        cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
        loadCloud(lidar_path, cloud);

//        projectToImage(cloud, img);
        projectToImage2(cloud, img);
    }
//    projectToImage2(cloud, img);

//    projectToImage1(cloud, img);
//    projectToImage(cloud, img);

//    deskewPoint(cloud[0]);
//    deskewPoint(cloud[cloud.size()-1]);


//    cv::Mat canvas = cv::Mat::zeros(2000, 2000, CV_8UC3);
//    for (auto &p: cloud) {
//        drawPoint(canvas, p, 255, 255, 255);
//    }

//    drawPoint2(canvas, cloud[0], 0, 255, 0);
//    drawPoint2(canvas, cloud[cloud.size()-1], 0, 255, 255);
//
//    show(canvas, 0);

//    std::cout << "size: " << cloud.size() << std::endl;

    return 0;

}
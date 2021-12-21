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

// linear velocity at time zero
const float vx = 6.98390467761731f;
const float vy = 0.18333130383767f;
const float vz = 0.15356837236718f;

// linear acceleration
const float ax = 1.18885125806239f;
const float ay = 0.75031638425215f;
const float az = 13.01972102311150f;

// angular velocity
const float wx = 0.17625848913349f;
const float wy = 0.19206552946943f;
const float wz = 0.02318471760338f;

Eigen::Transform<float, 3, 2> Tr, Tr_inverse;

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

void deskewPoint(pcl::PointXYZI &p) {
    double yaw = atan2(p.y, p.x);
    double dt = .05*yaw/(M_PI/2.0) -.05;

    double prev_vx = vx;// + ax*dt;
    double prev_vy = vy;// + ay*dt;
    double prev_vz = vz;// + az*dt;

    double dx = 0.005*ax + prev_vx*dt;
    double dy = 0.005*ay + prev_vy*dt;
    double dz = 0.005*az + prev_vz*dt;

    double orient_x = wx*dt;
    double orient_y = wy*dt;
    double orient_z = wz*dt;

    auto Trans = pcl::getTransformation(dx, dy, dz, orient_x, orient_y, orient_z);
//    auto Tr_inv = Tr.inverse();
//
//    auto TM = Tr_inv * Tr;

    double x = Trans(0,0) * p.x + Trans(0,1) * p.y + Trans(0,2) * p.z + Trans(0,3);
    double y = Trans(1,0) * p.x + Trans(1,1) * p.y + Trans(1,2) * p.z + Trans(1,3);
//    double z = Trans(2,0) * p.x + Trans(2,1) * p.y + Trans(2,2) * p.z + Trans(2,3);

    p.x = x;
    p.y = y;
//    p.z = z;

//    std::cout << "POS (dt= " << dt << ")  :  " << p.x << "  " << p.y << "  " << p.z << " )" << std::endl;

}

void projectToImage(std::vector<pcl::PointXYZI> &points, cv::Mat &img) {

    cv::Mat copy = img.clone();

    for (size_t i=0; i<points.size(); i++) {
        pcl::PointXYZI p = points[i];

        deskewPoint(p);

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
        dists[i] = points[i].x*points[i].x + points[i].y*points[i].y + points[i].z*points[i].z;
    }
    for (float dist = 20.0f; dist>=5.0f; dist-=0.1f) {
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
                color[2] = 255;
            }
        }
        std::cout << "DIST: " << dist << std::endl;

        cv::imwrite("/home/fusy/pointdeskewed1.png", copy);

        cv::Mat resized;
        cv::resize(copy, resized, cv::Size(img.cols * 2, img.rows * 2));

        cv::imshow("wind", resized);
        char c = cv::waitKey(0);
        if (c == 'q') break;
        if (c == 'p') dist += 0.2f;
    }
}

int main() {

    std::string img_path   = "/home/fusy/bags/00/image_02/data/0000000001.png";
    std::string lidar_path = "/home/fusy/bags/00/velodyne/000001.bin";

    std::vector<pcl::PointXYZI> cloud;
//    IMULoader imu;

    cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);
    loadCloud(lidar_path, cloud);

        projectToImage(cloud, img);
    projectToImage2(cloud, img);

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
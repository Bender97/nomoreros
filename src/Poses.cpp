//#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "IMULoader.h"

//#include <Eigen/QR>
//#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common_headers.h>
//#include <pcl/point_types.h>
//#include <pcl/range_image/range_image.h>
//#include <pcl/filters/filter.h>
//#include <pcl/filters/voxel_grid.h>

std::vector<cv::Mat> poses;
cv::Mat canvas;

int width = 2000;
int height = 2000;
cv::Mat Tr;
cv::Mat Tr_inv;

float curr_x = .0f, curr_y = .0f, curr_z = .0f;
float curr_vx = 6.98390467761731f, curr_vy=0.18333130383767f, curr_vz=0.15356837236718f;

double wx = 0.17625848913349f, wy = 0.19206552946943f, wz = 0.02318471760338f;

float px=(float)width/2.0f, py=(float)height/2.0f;

bool show(cv::Mat &img, int timeout) {
    cv::Mat copy;
    cv::resize(img, copy, cv::Size(1000, 1000));
    cv::imshow("prova", copy);
    if (cv::waitKey(timeout) == 'q') return false;
    return true;
}

void loadPoses() {
    std::string path = "/home/fusy/bags/00/poses.txt";
    std::string line;
    std::ifstream fin(path);
    assert(fin.is_open());

    std::stringstream ss;
    int cont = 0;
    while (getline(fin, line)) {
        cv::Mat transmat = cv::Mat::zeros(4, 4, CV_32FC1);
        ss =    std::stringstream(line);
        ss >> transmat.at<float>(0, 0);
        ss >> transmat.at<float>(0, 1);
        ss >> transmat.at<float>(0, 2);
        ss >> transmat.at<float>(0, 3);
        ss >> transmat.at<float>(1, 0);
        ss >> transmat.at<float>(1, 1);
        ss >> transmat.at<float>(1, 2);
        ss >> transmat.at<float>(1, 3);
        ss >> transmat.at<float>(2, 0);
        ss >> transmat.at<float>(2, 1);
        ss >> transmat.at<float>(2, 2);
        ss >> transmat.at<float>(2, 3);

        transmat.at<float>(3, 3) = 1.0f;

        cv::Mat temp = transmat * Tr;

        transmat = Tr_inv*temp;

        poses.push_back(transmat);
        cont++;
//        if (cont>500) break;
    }
    fin.close();
}

void drawPoses() {

    for (auto &pose: poses) {
        int x = (int) (pose.at<float>(0, 3) + (float)width/2.0f);
        int y = (int) (pose.at<float>(1, 3) + (float)height/2.0f);
//        int z = (int) pose.at<float>(2, 3);

        if (x>=0 && x<width && y>=0 && y<height) {
            auto &pixel = canvas.at<cv::Vec3b>(y, x);
            pixel[0] = 255;
            pixel[1] = 255;
            pixel[2] = 255;
        }
        if (!show(canvas, 5)) break;
    }

}

int main() {
    float data[16] = {4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02,
                      -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02,
                      9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01,
                      .0f, .0f ,0.0f, 1.0f};
    Tr = cv::Mat(4, 4, CV_32F, data);
    Tr_inv = Tr.inv();

    canvas = cv::Mat::zeros( height, width, CV_8UC3);

    loadPoses();

    size_t n_zero = 10;

    for (size_t i=0; i<poses.size(); i++) {
        std::string old_str = std::to_string(i);
        auto new_str = "/home/fusy/bags/00/oxts/data/" + std::string(n_zero - MIN(n_zero, old_str.length()), '0') + old_str + ".txt";

        IMULoader imu(new_str);

        /// COMPUTE delta position vector
        cv::Mat delta = cv::Mat::zeros(4, 1, CV_32F);
        delta.at<float>(0, 0) = ( 0.5f * (float)imu.ax*.01f + curr_vx*0.1f );     // s_0 is like considered zero (we're in imu frame)
        delta.at<float>(1, 0) = ( 0.5f * (float)imu.ay*.01f + curr_vy*0.1f );
        delta.at<float>(2, 0) = ( 0.5f * (float)imu.az*.01f + curr_vz*0.1f );

        /// UPDATE velocities using the accelerations
        curr_vx += ( (float)imu.ax * 0.1f );
        curr_vy += ( (float)imu.ay * 0.1f );
        curr_vz += ( (float)imu.az * 0.1f );

        /// INTEGRATE THE ROTATION FINDING THE CURRENT ONE (wrt to the first frame)
        wx += ( 0.1*imu.wx );
        wy += ( 0.1*imu.wy );
        wz += ( 0.1*imu.wz );

        /// get the pose in imu frame
        auto T = pcl::getTransformation(delta.at<float>(0, 0), delta.at<float>(1, 0), delta.at<float>(2, 0),
                                        (float) wx, (float) wy, (float) wz);

        /// translate the pose to cv::Mat
        cv::Mat imu_pose = cv::Mat::zeros(4, 4, CV_32F);
        for (int r=0; r<4; r++)
            for (int c=0; c<4; c++)
                imu_pose.at<float>(r, c) = T(r, c);
        imu_pose.at<float>(3, 3) = 1.0f;

        cv::Mat final_delta = Tr_inv * (imu_pose * Tr);

        curr_x += final_delta.at<float>(2, 0);
        curr_y += final_delta.at<float>(1, 0);

        int imu_pixel_x = (int) (px + curr_x );
        int imu_pixel_y = (int) (py + curr_y );

        if (imu_pixel_x>=0 && imu_pixel_x<width && imu_pixel_y>=0 && imu_pixel_y<height) {
            auto &pixel = canvas.at<cv::Vec3b>(imu_pixel_y, imu_pixel_x);
            pixel[0] = 0;
            pixel[1] = 0;
            pixel[2] = 255;
        }


        /// DRAW POSE got from GPS
        int pose_x = (int) (poses[i].at<float>(0, 3) + (float)width/2.0f);
        int pose_y = (int) ((float)height/2.0f - poses[i].at<float>(1, 3));
//        int z = (int) pose.at<float>(2, 3);

        if (pose_x>=0 && pose_x<width && pose_y>=0 && pose_y<height) {
            auto &pixel = canvas.at<cv::Vec3b>(pose_y, pose_x);
            pixel[0] = 255;
            pixel[1] = 255;
            pixel[2] = 255;
        }


        if (!show(canvas, 1)) break;

    }

//    drawPoses();



    show(canvas, 0);

    return 0;
}
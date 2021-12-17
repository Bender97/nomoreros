//
// Created by fusy on 12/12/21.
//

#ifndef PROJECTING_POINTCLOUDDESKEW_H
#define PROJECTING_POINTCLOUDDESKEW_H

#include <Eigen/QR>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include "Point.h"

class PointCloudDeskew {
public:
    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    double *imuTime = new double[2000];
    double *imuRotX = new double[2000];
    double *imuRotY = new double[2000];
    double *imuRotZ = new double[2000];

    std::deque<IMULoader> imuQueue;
//    std::deque<nav_msgs::Odometry> odomQueue;

    int deskewFlag;

    double timeScanCur;
    double timeScanEnd;

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur, float *rot);

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur);

    Point deskewPoint(Point *point, double relTime, float *rot);

    bool deskewInfo();
    void imuDeskewInfo();

};


#endif //PROJECTING_POINTCLOUDDESKEW_H

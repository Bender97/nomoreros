//
// Created by fusy on 10/12/21.
//

#pragma once
#ifndef PROJECTING_POINT_H
#define PROJECTING_POINT_H

class Point {
public:
    double x, y, z;
    Point(double _x, double _y, double _z) {
        x = _x;
        y = _y;
        z = _z;
    }
    Point() {};
};


#endif //PROJECTING_POINT_H

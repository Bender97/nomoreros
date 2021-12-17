#pragma once

#ifndef TRAV_ANALYSIS_DATA_H
#define TRAV_ANALYSIS_DATA_H

class Data {
public:
    const float resolution = 0.4; // meters
    const float resolution_squared = resolution*resolution; // meters
    const float half_resolution = resolution / 2; // meters
    const int32_t grid_side_length = 60;
    const int32_t grid_side_length_squared = grid_side_length * grid_side_length;
    const int32_t half_grid_side_length = grid_side_length / 2;

    const float internal_resolution = 0.2; // meters
    const int32_t internal_grid_side_length = (int32_t) (resolution / internal_resolution);
    const int32_t internal_grid_side_length_squared = internal_grid_side_length * internal_grid_side_length;
    const int32_t internal_half_grid_side_length = internal_grid_side_length / 2;

    const std::string datapath = "/home/fusy/bags/00/velodyne/";
    const std::string labelspath = "/home/fusy/bags/00/labels/";

    const std::string svm_model_path = "/home/fusy/repos/project/svm_model/rbf_svm.yml";
    const std::string norm_path = "/home/fusy/repos/project/svm_model/norm_params.txt";

    const int height = 1500;
    const int width = 1500;
    const int px_per_m = 75 ;

    const int sq_px = (int) round(0.4f * px_per_m);
    const int sq_px_half = (int) round(sq_px / 2);

    const float gbr_x = -half_grid_side_length * resolution - resolution / 2.0f;
    const float gbr_y = -half_grid_side_length * resolution - resolution / 2.0f;

    std::map<int, std::array<int, 3> > color_map = {
            {0,   std::array<int, 3>{0, 0, 0}},
            {1,   std::array<int, 3>{0, 0, 255}},
            {10,  std::array<int, 3>{245, 150, 100}},
            {11,  std::array<int, 3>{245, 230, 100}},
            {13,  std::array<int, 3>{250, 80, 100}},
            {15,  std::array<int, 3>{150, 60, 30}},
            {16,  std::array<int, 3>{255, 0, 0}},
            {18,  std::array<int, 3>{180, 30, 80}},
            {20,  std::array<int, 3>{255, 0, 0}},
            {30,  std::array<int, 3>{30, 30, 255}},
            {31,  std::array<int, 3>{200, 40, 255}},
            {32,  std::array<int, 3>{90, 30, 150}},
            {40,  std::array<int, 3>{255, 0, 255}},
            {44,  std::array<int, 3>{255, 150, 255}},
            {48,  std::array<int, 3>{75, 0, 75}},
            {49,  std::array<int, 3>{75, 0, 175}},
            {50,  std::array<int, 3>{0, 200, 255}},
            {51,  std::array<int, 3>{50, 120, 255}},
            {52,  std::array<int, 3>{0, 150, 255}},
            {60,  std::array<int, 3>{170, 255, 150}},
            {70,  std::array<int, 3>{0, 175, 0}},
            {71,  std::array<int, 3>{0, 60, 135}},
            {72,  std::array<int, 3>{80, 240, 150}},
            {80,  std::array<int, 3>{150, 240, 255}},
            {81,  std::array<int, 3>{0, 0, 255}},
            {99,  std::array<int, 3>{255, 255, 50}},
            {252, std::array<int, 3>{245, 150, 100}},
            {256, std::array<int, 3>{255, 0, 0}},
            {253, std::array<int, 3>{200, 40, 255}},
            {254, std::array<int, 3>{30, 30, 255}},
            {255, std::array<int, 3>{90, 30, 150}},
            {257, std::array<int, 3>{250, 80, 100}},
            {258, std::array<int, 3>{180, 30, 80}},
            {259, std::array<int, 3>{255, 0, 0}},
    };

    // original projection matrix
//    const double P200 = 7.18856e+02;
//    const double P201 = 0.0e+00;
//    const double P202 = 6.071928e+02;
//    const double P203 = 4.538225e+01;
//    const double P210 = 0.0e+00;
//    const double P211 = 7.18856e+02;
//    const double P212 = 1.852157e+02;
//    const double P213 = -1.130887e-01;
//    const double P220 = 0.0e+00;
//    const double P221 = 0.0e+00;
//    const double P222 = 1.0e+00;
//    const double P223 = 3.779761e-03;

    // taken from internet
    const double P200 = 7.070493000000e+02;
    const double P201 = 0.000000000000e+00;
    const double P202 = 6.040814000000e+02;
    const double P203 = 0.000000000000e+00;
    const double P210 = 0.000000000000e+00;
    const double P211 = 7.070493000000e+02;
    const double P212 = 1.805066000000e+02;
    const double P213 = 0.000000000000e+00;
    const double P220 = 0.0e+00;
    const double P221 = 0.0e+00;
    const double P222 = 1.0e+00;
    const double P223 = 0;


    // transformation matrix taken from "the original kitti paper" - thesis
    const double Tr00 = 6.927964000000e-03;
    const double Tr01 = -9.999722000000e-01;
    const double Tr02 = -2.757829000000e-03;
    const double Tr03 = -2.457729000000e-02;
    const double Tr10 = -1.162982000000e-03;
    const double Tr11 = 2.749836000000e-03;
    const double Tr12 = -9.999955000000e-01;
    const double Tr13 = -6.127237000000e-02;
    const double Tr20 = 9.999753000000e-01;
    const double Tr21 = 6.931141000000e-03;
    const double Tr22 = -1.143899000000e-03;
    const double Tr23 = -3.321029000000e-01;
    const double Tr30 = 0;
    const double Tr31 = 0;
    const double Tr32 = 0;
    const double Tr33 = 1;

    // transformation matrix taken from the calib file of raw kitti dataset
//    const double Tr00 = 7.533745e-03;
//    const double Tr01 = -9.999714e-01 ;
//    const double Tr02 = -6.166020e-04 ;
//    const double Tr03 = -4.069766e-03 ;
//    const double Tr10 = 1.480249e-02 ;
//    const double Tr11 = 7.280733e-04 ;
//    const double Tr12 = -9.998902e-01 ;
//    const double Tr13 = -7.631618e-02 ;
//    const double Tr20 = 9.998621e-01 ;
//    const double Tr21 = 7.523790e-03 ;
//    const double Tr22 = 1.480755e-02 ;
//    const double Tr23 = -2.717806e-01;
//    const double Tr30 = 0;
//    const double Tr31 = 0;
//    const double Tr32 = 0;
//    const double Tr33 = 1;
};

#endif // TRAV_ANALYSIS_DATA_H
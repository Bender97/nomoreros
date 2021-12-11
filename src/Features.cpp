#include "Features.h"

void Features::reset() {
    linearity       = .0f; planarity        = .0f; sphericity         = .0f; omnivariance      = .0f;
    anisotropy      = .0f; eigenentropy     = .0f; sum_of_eigenvalues = .0f; curvature         = .0f;
    angle           = .0f; goodness_of_fit  = .0f; roughness          = .0f; average_elevation = .0f;
    normal_vector.x = .0f; normal_vector.y  = .0f; normal_vector.z    = .0f;
    unevenness      = .0f; surface_density  = .0f;
    z_diff          = .0f; internal_density = .0f; curvity            = .0f; label             =   0;
    color_hsv[0]    =   0; color_hsv[1]     =   0;  color_hsv[2]      =   0;
}

template<typename CustomPoint>
void Features::computeCorrelationMatrixComponents(std::vector<CustomPoint *> cell) {

    for (auto point: cell) {
        cx += point->x;
        cy += point->y;
        cz += point->z;
    }
    cx /= (float) numpoints; cy /= (float) numpoints;  cz /= (float) numpoints;


    for (auto point: cell) {
        float ax = point->x - cx;
        float ay = point->y - cy;
        float az = point->z - cz;

        a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
        a22 += ay * ay; a23 += ay * az;
        a33 += az * az;
    }
    a11 /= (float) numpoints; a12 /= (float) numpoints; a13 /= (float) numpoints; a22 /= (float) numpoints; a23 /= (float) numpoints; a33 /= (float) numpoints;

    matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
    matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
    matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;
}

void Features::computeCorrelationMatrixComponents(std::vector<Point> &cell) {
    cx = 0, cy = 0, cz = 0;
    a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
    numpoints = cell.size();
    for (auto point: cell) {
        cx += point.x;
        cy += point.y;
        cz += point.z;
    }
    cx /= (float) numpoints; cy /= (float) numpoints;  cz /= (float) numpoints;


    for (auto point: cell) {
        float ax = point.x - cx;
        float ay = point.y - cy;
        float az = point.z - cz;

        a11 += ax * ax; a12 += ax * ay; a13 += ax * az;
        a22 += ay * ay; a23 += ay * az;
        a33 += az * az;
    }
    a11 /= (float) numpoints; a12 /= (float) numpoints; a13 /= (float) numpoints; a22 /= (float) numpoints; a23 /= (float) numpoints; a33 /= (float) numpoints;

    matA1.at<float>(0, 0) = a11; matA1.at<float>(0, 1) = a12; matA1.at<float>(0, 2) = a13;
    matA1.at<float>(1, 0) = a12; matA1.at<float>(1, 1) = a22; matA1.at<float>(1, 2) = a23;
    matA1.at<float>(2, 0) = a13; matA1.at<float>(2, 1) = a23; matA1.at<float>(2, 2) = a33;
}



bool Features::computeGeometricalFeatures( const std::vector<Point *> &points_in_bucket,
                                           cv::Point3f &scene_normal,
                                           Point &belonging_cell_center,
                                           int numbuckets) {
    cx = 0, cy = 0, cz = 0;
    a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
    numpoints = points_in_bucket.size();
    // compute eigenvalues and eigenvectors
    matA1 = cv::Mat::zeros(cv::Size(3, 3), CV_32F);
    matD1 = cv::Mat::zeros(cv::Size(1, 3), CV_32F);
    matV1 = cv::Mat::zeros(cv::Size(3, 3), CV_32F);
    computeCorrelationMatrixComponents(points_in_bucket);
    cv::eigen(matA1, matD1, matV1);

    // store eigenvalues in an easy way
    d1 = matD1.at<float>(0, 0);
    d2 = matD1.at<float>(0, 1);
    d3 = matD1.at<float>(0, 2);

    if (d1==0 || d2==0 || d3==0) return false;
//    if (points_in_bucket.empty()) return false;

    cv::Point3f origin(  cx, cy, cz  );
    normal_vector = cv::Point3f(matV1.at<float>(2, 0), matV1.at<float>(2, 1), matV1.at<float>(2, 2));

    /// COVARIANCE-BASED
    linearity = (d1 - d2) / d1;
    planarity = (d2 - d3) / d1;
    sphericity = d3 / d1;
    omnivariance = std::cbrt(d1*d2*d3);
    anisotropy = (d1 - d3) / d1;
    eigenentropy = - ( d1*std::log(d1) + d2*std::log(d2) + d3*std::log(d3) );
    sum_of_eigenvalues = d1 + d2 + d3;
    curvature = d3 / sum_of_eigenvalues;

    /// ROUGHNESS-BASED
    float d = - SCALAR_PRODUCT(normal_vector, origin) ;                   // scalar product
    float normal_magnitude = MAGNITUDE(normal_vector);
    if (normal_magnitude == 0) return false;

    angle = std::acos(normal_vector.z);    // normal.z = normal.x*0 + normal.y*0 + normal.z*1, with z = (0, 0, 1)
//    goodness_of_fit = .0f;
    for (auto p : points_in_bucket) { goodness_of_fit += std::abs(SCALAR_PRODUCT_2p(normal_vector, p) + d) / normal_magnitude;}
    for (auto p : points_in_bucket) roughness += NUM2SQ(p->z - origin.z);
    roughness /= (float) numpoints;
    average_elevation = origin.z;

    /// NORMAL VECTOR-BASED
//    normal_vector = cv::Point3f(normal.x, normal.y, normal.z);
    // feature.sum_of_vectors is not implemented (only one normal vector per cell)
    unevenness = normal_magnitude / (float) numpoints;
    surface_density = (float) numpoints / (data.resolution_squared);

    /// Z_DIFF feature
    float min = points_in_bucket[0]->z, max = points_in_bucket[0]->z;
    for (auto p : points_in_bucket) {
        float local_d = - SCALAR_PRODUCT_2p(scene_normal, p);
        float z = - (scene_normal.x*origin.x + scene_normal.y*origin.y - NUM2SQ(scene_normal.x)/scene_normal.z*origin.z
                     - NUM2SQ(scene_normal.y)/scene_normal.z*origin.z + local_d)
                  / (NUM2SQ(scene_normal.x)/scene_normal.z + NUM2SQ(scene_normal.y)/scene_normal.z + scene_normal.z);
        if (z>max) max = z;
        else if (z<min) min = z;
    }
    z_diff = max-min;       // I think it should be always positive


    /// COMPUTE INTERNAL DENSITY
    bool internal[data.internal_grid_side_length_squared];
    int internal_idx;


    // init data
    for (int i=0; i<data.internal_grid_side_length_squared; i++) internal[i] = false;
    int occupied_cells_num = 0;

    // count how many internal cells are not empty
    for (auto &point: points_in_bucket) {

        int row = floor(((belonging_cell_center.y + data.half_resolution - point->y) / data.resolution) * data.internal_grid_side_length);
        int col = floor(((belonging_cell_center.x + data.half_resolution - point->x) / data.resolution) * data.internal_grid_side_length);

        if (row==data.internal_grid_side_length && (fabs(belonging_cell_center.y - point->y) - data.half_resolution  <= 1e-5 )) {
            row=data.internal_grid_side_length-1;
        }
        if (col==data.internal_grid_side_length && (fabs(belonging_cell_center.x - point->x - data.half_resolution) <= 1e-5 )) {
            col=data.internal_grid_side_length-1;
        }

        internal_idx = row * data.internal_grid_side_length + col;
        if (!(internal[internal_idx])) {
            occupied_cells_num++;       // if internal cell is empty (never counted yet)
            internal[internal_idx] = true;
        }
    }
    // compute the density over the occupied cells
    internal_density = (float) points_in_bucket.size() / (float) occupied_cells_num;


    /// COMPUTE CURVITY FEATURES
    float max_magn = -1, min_magn = FLT_MAX;
    std::vector<float> magns(points_in_bucket.size());

    for (size_t i=0; i<points_in_bucket.size(); i++) {
        magns[i] = NUM2SQ(points_in_bucket[i]->x) + NUM2SQ(points_in_bucket[i]->y) + NUM2SQ(points_in_bucket[i]->z);
        if (magns[i]>max_magn) max_magn = magns[i];
        if (magns[i]<min_magn) min_magn = magns[i];
    }

    std::vector<short int> bucket = std::vector<short int>(numbuckets, 0);
    float step = (max_magn - min_magn) / (float)numbuckets;

    for (float magn: magns) {
        int idx = floor((magn - min_magn) / step);
        bucket[idx] ++;
    }

    curvity = 0;
    for (int buck: bucket) if (buck==0) curvity ++;


//    if (goodness_of_fit > 30.0f)  goodness_of_fit     = 30.0f;
    if (roughness > 1.0f)         goodness_of_fit     = 1.0f;
//    if (surface_density > 2000)   surface_density     = 2000;
//    if (internal_density > 250)   internal_density    = 250;
    if (z_diff > 0.4)             z_diff              = 0.4;
    if (sum_of_eigenvalues > 0.5) sum_of_eigenvalues  = 0.5;


    goodness_of_fit = ( goodness_of_fit > 0 ? std::log(goodness_of_fit) : goodness_of_fit );
//    roughness = ( roughness > 0 ? std::log(roughness) : roughness );
    surface_density = ( surface_density > 0 ? std::log(surface_density) : surface_density );
    internal_density = ( internal_density > 0 ? std::log(internal_density) : internal_density );
//    sum_of_eigenvalues = ( sum_of_eigenvalues > 0 ? std::log(sum_of_eigenvalues) : sum_of_eigenvalues );
//    curvity = ( curvity > 0 ? std::log(curvity) : curvity );

    return true;
}



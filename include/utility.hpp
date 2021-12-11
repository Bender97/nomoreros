//
// Created by fusy on 10/12/21.
//

#pragma once
#ifndef PROJECTING_UTILITY_HPP
#define PROJECTING_UTILITY_HPP


void loadLidarScans(std::string lidarpath, std::vector<Point> &scans) {
    float f;
    std::ifstream fin(lidarpath, std::ios::binary);
    Point p;
    int cont = 0;

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
                scans.push_back(p);
                break;
        }
        cont = (cont+1)%4;
    }
    fin.close();
}

void loadLabels(std::string labelpath, std::vector<int32_t> &labels) {
    int32_t lab;
    std::ifstream lin(labelpath, std::ios::binary);
    while (lin.read(reinterpret_cast<char*>(&lab), sizeof(int32_t))) {
        labels.push_back(lab);
    }
    lin.close();
}

cv::Point2f projectPoint(double x, double y, double z, Data &data) {
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


void sortPointsInGrid(std::vector<Point> &cloudToSort, std::vector<std::vector<Point*>> &grid_buckets, Data &data) {
    int cell_idx;
    Point *p;
    grid_buckets.clear();
    grid_buckets.resize(data.grid_side_length_squared);

    for (size_t i=0; i<cloudToSort.size(); i++) {

        // extract which cell of the traversability grid the point of the cloud belongs to
        p = &(cloudToSort[i]);
        int col = (int) round((p->x - data.gbr_x) / data.resolution);
        int row = (int) round((p->y - data.gbr_y) / data.resolution);

        if (row>=0 && row < data.grid_side_length && col >= 0 && col < data.grid_side_length) {
            cell_idx = row * data.grid_side_length + col;
            // add the point to its cell
            grid_buckets[cell_idx].push_back(p);

//            int img_row = int(p->y*px_per_m) + height/2;
//            int img_col = int(p->x*px_per_m) + width/2;
//
//            if (img_row>=0 && img_row<height && img_col>=0 && img_col < width) {
//                cv::Vec3b & color = img.at<cv::Vec3b>(img_row, img_col);
//                color[0] = (( row * 13 )%139 + (col * 171) % 1023) % 255;
//                color[1] = (( row * 167 )%17 + (col * 1712) % 1023) % 255;
//                color[2] = (( row * 1453 )%139 + (col * 19) % 1023) % 255;
//            }
        }
    }

}


void projectToImage(std::vector<Point> &points , std::vector<int> &labels, Data &data) {
    cv::Mat img = cv::imread("../imgs/img000000.png", cv::IMREAD_COLOR);

    for (size_t i=0; i<points.size(); i++) {
        Point p = points[i];
        if (p.x<0) continue;
        int32_t l = labels[i];
        cv::Point2f scaled = projectPoint(p.x, p.y, p.z, data);
        if (scaled.x >= 0 && scaled.x<img.cols && scaled.y>=0 && scaled.y<img.rows) {
            cv::Vec3b & color = img.at<cv::Vec3b>((int)scaled.y, (int)scaled.x);
            color[0] = data.color_map[l][0];
            color[1] = data.color_map[l][1];
            color[2] = data.color_map[l][2];
        }
    }
    cv::Mat resized;
    cv::resize(img, resized, cv::Size(img.cols*2, img.rows*2));

    cv::imshow("wind", resized);
    cv::waitKey(0);
}

void loadNormalizationConfig(const std::string &normalization_config_path,
                                      int features_num,
                                      std::vector<float> &min,
                                      std::vector<float> &p2p) {

    std::ifstream input_normalization_config_file;
    input_normalization_config_file = std::ifstream(normalization_config_path);
    std::string line;
    size_t start, end;
    int col_cont;

    min.resize(features_num);
    p2p.resize(features_num);


    // load min
    start = 0, col_cont = 0;
    getline (input_normalization_config_file, line);
    assert(line.length()>0);
    for (size_t i=0; i<line.length(); i++) {
        assert(col_cont<features_num);
        if (line.at(i) == ' ' || line.at(i) == '\n' || i==line.length()-1) {
            end = i;
            min[col_cont] = (float) std::stod(line.substr(start, end));
            start = i;
            col_cont++;
        }
    }

    // load p2p
    start = 0, col_cont = 0;
    getline (input_normalization_config_file, line);
    assert(line.length()>0);
    for (size_t i=0; i<line.length(); i++) {
        assert(col_cont<features_num);
        if (line.at(i) == ' ' || line.at(i) == '\n' || i==line.length()-1) {
            end = i;
            p2p[col_cont] = (float) std::stod(line.substr(start, end));
            start = i;
            col_cont++;
        }
    }

    if (input_normalization_config_file.is_open())
        input_normalization_config_file.close();
}


bool getCellFeaturesPred(std::vector<Point *> &points, Features &feature, Point &belonging_cell_center,
                         cv::Point3f &scene_normal
) {

    if (points.size() < 10) return false;

    if (!feature.computeGeometricalFeatures(points, scene_normal, belonging_cell_center, 40)) {
        return false;
    }


    return true;
}

/// calculate normal of the whole scene
void computeScenePlaneNormal(std::vector<Point> &cloud, Features &feature, cv::Point3f &scene_normal) {

    feature.matA1 = cv::Mat::zeros(cv::Size(3, 3), CV_32F);
    feature.matD1 = cv::Mat::zeros(cv::Size(1, 3), CV_32F);
    feature.matV1 = cv::Mat::zeros(cv::Size(3, 3), CV_32F);

    feature.computeCorrelationMatrixComponents(cloud);
    cv::eigen(feature.matA1, feature.matD1, feature.matV1);

    scene_normal.x = feature.matV1.at<float>(2, 0);
    scene_normal.y = feature.matV1.at<float>(2, 1);
    scene_normal.z = std::abs(feature.matV1.at<float>(2, 2));
}

bool columnIsRequested(int col, std::vector<int> &skipColWithIndex) {
    for (const auto c: skipColWithIndex) if (col==c) return false;
    return true;
}

float getAvgElevationOfPointsInCell(std::vector<Point *> cell) {
    float z_sum = .0f;
    for (auto &point: cell) z_sum += point->z;
    return ( z_sum / ( (float) cell.size() ) );
}


#endif //PROJECTING_UTILITY_HPP

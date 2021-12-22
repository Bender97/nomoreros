#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include <map>
#include "data.h"
#include "Features.h"
#include "Point.h"
#include "utility.hpp"
#include <cstdlib>

using namespace std;


Data data;
Features feature;
// SVM
cv::Mat features_matrix, predictions_vector;
std::vector<int> cellIsPredictable;

int features_num;
std::vector<int> feats_indexes;
std::vector<float> min_vals, p2p_vals;         // needed to normalize new data
std::vector<int> skipColWithIndex({11, 20, 21});

void draw(std::vector<Point> &scan_points, std::vector<int> &labels, std::vector<Point> &grid) {
    cv::Mat img = cv::Mat::zeros(data.height, data.width, CV_8UC3);
    for (int featidx=0; featidx<features_matrix.cols; featidx++) {

        // draw just the cloud points
        for (size_t i = 0; i < scan_points.size(); i++) {
            Point p = scan_points[i];

            int row = int(p.y * data.px_per_m) + data.height / 2;
            int col = int(p.x * data.px_per_m) + data.width / 2;

            if (row >= 0 && row < data.height && col >= 0 && col < data.width) {
                int32_t l = labels[i];
                auto &color = img.at<cv::Vec3b>(row, col);
                color[0] = data.color_map[l][0];
                color[1] = data.color_map[l][1];
                color[2] = data.color_map[l][2];
            }

        }

        // draw the rectangle of the
        for (int i = 0; i < data.grid_side_length_squared; i++) {
            Point p = grid[i];

            int row = int(p.y * data.px_per_m) + data.height / 2;
            int col = int(p.x * data.px_per_m) + data.width / 2;

            if (row >= 0 && row < data.height && col >= 0 && col < data.width) {
                auto &color = img.at<cv::Vec3b>(row, col);
                color[0] = 0;
                color[1] = 0;
                color[2] = 255;

                float featval = features_matrix.at<float>(i, featidx);

                cv::Rect rect(col - data.sq_px_half, row - data.sq_px_half, data.sq_px, data.sq_px);
                cv::rectangle(img, rect, cv::Scalar(
                                      (int) (featval * 1721) % 255,
                                      (int) (featval * 1721) % 255,
                                      (int) (featval * 173) % 255),
                              3);

            }

        }

        std::cout << "SHOWING FEAT " << featidx << " / " << features_matrix.cols << std::endl;
        cv::imshow("wind", img);
        cv::waitKey(0);
    }
}


void initGrid(std::vector<Point> &grid) {


    features_matrix = cv::Mat::zeros(data.grid_side_length_squared, features_num, CV_32F);
    predictions_vector = cv::Mat::zeros(data.grid_side_length_squared, 1, CV_32FC1);
    cellIsPredictable.resize(data.grid_side_length_squared);
	grid.resize(data.grid_side_length_squared);

	float x, y;
    for (int col=0; col<data.grid_side_length; ++col) {
        for (int row = 0; row < data.grid_side_length; ++row) {
            int idx = row * data.grid_side_length + col;
            x = (float) col * data.resolution + data.gbr_x;
            y = (float) row * data.resolution + data.gbr_y;
            grid[idx].x = x;
            grid[idx].y = y;
            cellIsPredictable[idx] = 0;
        }
    }
}

void buildFeature(cv::Mat &x) {
    float *start = &(feature.linearity);

    for (int i=0; i < (int) feats_indexes.size()    ; ++i)
        x.at<float>(0, i) =  (*(start + feats_indexes[i]) - min_vals[i]) / p2p_vals[i];
}

void fillFeatureMatrix( std::vector<Point> &points, std::vector<Point> &grid, std::vector< std::vector<Point*> > &grid_buckets ) {

    // feature vector where the features are copied once, then recopied to feature matrix (each row pass by here)
    cv::Mat feature_vector(1, features_num, CV_32F);

    /// calculate normal of the whole scene
    cv::Point3f scene_normal;
    computeScenePlaneNormal(points, feature, scene_normal);

    /// build the feature matrix x (in the indexes vector store the correspondant index of the cell)
    for (int i = 0; i < data.grid_side_length_squared; ++i) {
        feature.reset();

        /// if this function returns false, then the class should be unknown (already set as default)
        if ( getCellFeaturesPred(grid_buckets[i], feature, grid[i], scene_normal) ) {

            /// build the feature vector corresponding to the current field cell
            buildFeature(feature_vector);
            /// copy the row
            for (int j=0; j<features_num; j++) features_matrix.at<float>(i, j) = feature_vector.at<float>(0, j);

            cellIsPredictable[i] = 1;
        }
    }
}


void loadFeaturesData() {
    features_num = 19;

    // stack the indexes of the features that svm will use
    for (int i=0; i<22; ++i) {
        if(columnIsRequested(i, skipColWithIndex))
            feats_indexes.push_back(i);
    }
    loadNormalizationConfig(data.norm_path, features_num, min_vals, p2p_vals);
}

void projectGridToImage(std::vector< std::vector<Point*> > &grid_buckets, std::vector<Point> &gridPoints , std::vector<int> &labels, Data &data) {
    cv::Mat img = cv::imread("../imgs/img000000.png", cv::IMREAD_COLOR);

    std::vector<cv::Scalar> colors(grid_buckets.size());
    for (auto &color: colors) color = cv::Scalar((double)std::rand() / RAND_MAX * 255,
                                                 (double)std::rand() / RAND_MAX * 255,
                                                 (double)std::rand() / RAND_MAX * 255);

    for (int bucketidx=0; bucketidx<grid_buckets.size(); bucketidx++) {

        auto bucket = grid_buckets[bucketidx];

        for (size_t i = 0; i < bucket.size(); i++) {
            Point *p = bucket[i];
            if (p->x < 0) continue;

            int32_t l = labels[i];
            cv::Point2f scaled = projectPoint(p->x, p->y, p->z, data);
            if (scaled.x >= 0 && scaled.x < img.cols && scaled.y >= 0 && scaled.y < img.rows) {
                cv::Vec3b &color = img.at<cv::Vec3b>((int) scaled.y, (int) scaled.x);
                color[0] = colors[bucketidx][0];
                color[1] = colors[bucketidx][1];
                color[2] = colors[bucketidx][2];

            }
        }
    }

    for (size_t i=0; i<gridPoints.size(); i++) {
        Point p = gridPoints[i];
        if (p.x<0) continue;

        cv::Point2f tl = projectPoint(p.x+0.2f, p.y+0.2f, p.z, data);
        cv::Point2f tr = projectPoint(p.x+0.2f, p.y-0.2f, p.z, data);
        cv::Point2f bl = projectPoint(p.x-0.2f, p.y+0.2f, p.z, data);
        cv::Point2f br = projectPoint(p.x-0.2f, p.y-0.2f, p.z, data);


        int32_t l = labels[i];
        cv::Point2f scaled = projectPoint(p.x, p.y, p.z, data);
        if (scaled.x >= 0 && scaled.x<img.cols && scaled.y>=0 && scaled.y<img.rows) {
            cv::Vec3b & color = img.at<cv::Vec3b>((int)scaled.y, (int)scaled.x);
            color[0] = data.color_map[l][0];
            color[1] = data.color_map[l][1];
            color[2] = data.color_map[l][2];

            cv::line(img, tl, tr, colors[i], 1);
            cv::line(img, tr, br, colors[i], 1);
            cv::line(img, br, bl, colors[i], 1);
            cv::line(img, bl, tl, colors[i], 1);

        }
    }
    cv::Mat resized;
    cv::resize(img, resized, cv::Size(img.cols*2, img.rows*2));

    cv::imshow("wind", resized);
    cv::waitKey(0);
}

void updateGridCellsElevation(std::vector< std::vector<Point*> > &grid_buckets, std::vector<Point> &grid) {
    for (int cell_idx=0; cell_idx<data.grid_side_length_squared; ++cell_idx) {
        /// if the cell has a suff num of points, set the avg elev to the pred cell
        if (grid_buckets[cell_idx].size() >= 10)
            grid[cell_idx].z = getAvgElevationOfPointsInCell(grid_buckets[cell_idx]);
            /// else remove points which are unknown (only for visualization)
        else {
            grid[cell_idx].z         = HIGHEST_Z_VALUE;
        }
    }
}

int main() {

    std::srand(time(0));
	std::vector<Point> grid;
    std::vector<Point> points;
    std::vector< std::vector<Point*> > grid_buckets;
    std::vector<int> labels;
    loadFeaturesData();


	initGrid(grid);

	string lidarpath = data.datapath + "000000.bin";
	string labelpath = data.labelspath + "000000.label";

    // load data from files
	loadLidarScans(lidarpath, points);
    loadLabels(labelpath, labels);


    sortPointsInGrid(points, grid_buckets, data);
    updateGridCellsElevation(grid_buckets, grid);

    fillFeatureMatrix(points, grid, grid_buckets);

//    projectToImage(points, labels, data);
    projectGridToImage(grid_buckets, grid, labels, data);

//    draw(points, labels, grid);

//    projectToImage(grid, labels, data);
//    projectToImage(points, labels, data);


	return 0;
}
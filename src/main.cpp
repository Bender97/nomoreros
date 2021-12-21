#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include <map>
#include "data.h"
#include "Features.h"
#include "Point.h"
#include "utility.hpp"
#include "IMULoader.h"
//#include "PointCloudDeskew.h"
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

	cv::Mat img = cv::Mat(data.height, data.width, CV_8UC3, cv::Scalar(0, 0, 0));

	for (size_t i = 0; i<scan_points.size(); i++) {
		Point p = scan_points[i];

		int row = int(p.y*data.px_per_m) + data.height/2;
		int col = int(p.x*data.px_per_m) + data.width/2;

		if (row>=0 && row<data.height && col>=0 && col < data.width) {
			int32_t l = labels[i];
			auto & color = img.at<cv::Vec3b>(row, col);
			color[0] = data.color_map[l][0];
			color[1] = data.color_map[l][1];
			color[2] = data.color_map[l][2];
			color[0] = 0;
			color[1] = 0;
			color[2] = 255;
		}

	}

	for (int i = 0; i<data.grid_side_length_squared; i++) {
		Point p = grid[i];

		int row = int(p.y*data.px_per_m) + data.height/2;
		int col = int(p.z*data.px_per_m) + data.width/2;

		if (row>=0 && row<data.height && col>=0 && col < data.width) {
			auto & color = img.at<cv::Vec3b>(row, col);
			color[0] = 0;
			color[1] = 0;
			color[2] = 255;

			cv::Rect rect(col-data.sq_px_half, row-data.sq_px_half, data.sq_px, data.sq_px);
			cv::rectangle(img, rect, cv::Scalar(int(features_matrix.at<float>(i, 10) *1721) % 255,
                                                int(features_matrix.at<float>(i, 10) *1721) % 255,
                                                int(features_matrix.at<float>(i, 10)  *173) % 255),
                                            3);

		}

	}

	cv::imshow("wind", img);
	cv::waitKey(0);
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


void projectWithColors(std::vector<Point> &points, std::string img_path) {
    cv::Mat canvas = cv::Mat(data.height, data.width, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);

    for (size_t i = 0; i<points.size(); i++) {
        Point p = points[i];

        int canvas_row = data.height - (int(p.z*data.px_per_m) + data.height/2);
        int canvas_col = data.width - (int(p.y*data.px_per_m) + data.width/2);

        if (canvas_row<0 || canvas_row>=data.height || canvas_col<0 || canvas_col >= data.width) continue;
        auto & canvas_color = canvas.at<cv::Vec3b>(canvas_row, canvas_col);

        if (p.x<0) {
            canvas_color[0] = 0;
            canvas_color[1] = 0;
            canvas_color[2] = 255;
        }
        else {
            cv::Point2f scaled = projectPoint(p.x, p.y, p.z, data);
            if (scaled.x >= 0 && scaled.x < (float)img.cols && scaled.y >= 0 && scaled.y < (float)img.rows) {
                auto &img_color = img.at<cv::Vec3b>((int) scaled.y, (int) scaled.x);
                canvas_color[0] = img_color[0];
                canvas_color[1] = img_color[1];
                canvas_color[2] = img_color[2];
            }
            else {
                canvas_color[0] = 0;
                canvas_color[1] = 0;
                canvas_color[2] = 255;
            }
        }




    }

    cv::imshow("wind", canvas);
    cv::waitKey(0);
}

int main() {

        std::string path = "/home/fusy/bags/2011_09_26_drive_0002_sync/2011_09_26/2011_09_26_drive_0002_sync/oxts/data/0000000000.txt";
    IMULoader imu_loader(path);
//    imu_loader.loadIMU_from_file();

	std::vector<Point> grid;
    std::vector<Point> points;
    std::vector< std::vector<Point*> > grid_buckets;
    std::vector<int> labels;
    loadFeaturesData();


	initGrid(grid);

	string lidarpath = data.datapath + "000000.bin";
	string labelpath = data.labelspath + "000000.label";

    lidarpath="/home/fusy/bags/2011_09_26_drive_0002_sync/2011_09_26/2011_09_26_drive_0002_sync/velodyne_points/data/0000000000.bin";

    // load data from files
	loadLidarScans(lidarpath, points);
//    loadLabels(labelpath, labels);


//    sortPointsInGrid(points, grid_buckets, data);
//    updateGridCellsElevation(grid_buckets, grid);

//    fillFeatureMatrix(points, grid, grid_buckets);

    //draw(points, labels, grid);

//    PointCloudDeskew pcd;
//    float rot[3];
//    rot[0] = imu_loader.wx;
//    rot[1] = imu_loader.wy;
//    rot[2] = imu_loader.wz;
//
//    projectWithColors(points,  "/home/fusy/bags/2011_09_26_drive_0002_sync/2011_09_26/2011_09_26_drive_0002_sync/image_03/data/0000000000.png");
//    for (size_t i=0; i<points.size(); i++) {
//        pcd.deskewPoint(&(points[i]), .0, rot);
//    }

//    projectToImage(grid, labels, data);
//    projectToImage(points, labels, data);

//    projectWithColors(points,  "/home/fusy/bags/2011_09_26_drive_0002_sync/2011_09_26/2011_09_26_drive_0002_sync/image_03/data/0000000000.png");


    for (int i=0; ; i++) {
        std::string s = std::to_string(i);
        s.insert(s.begin(), 10 - s.size(), '0');

        lidarpath="/home/fusy/bags/2011_09_26_drive_0002_sync/2011_09_26/2011_09_26_drive_0002_sync/velodyne_points/data/" + s + ".bin";
        std::string img_path = "/home/fusy/bags/2011_09_26_drive_0002_sync/2011_09_26/2011_09_26_drive_0002_sync/image_03/data/" + s + ".png";
        loadLidarScans(lidarpath, points);
        projectWithColors(points, img_path);
    }


	return 0;
}
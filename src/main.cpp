#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/ml.hpp>
#include <map>
#include "data.h"
#include "Features.h"
#include "Point.h"
#include "utility.hpp"

using namespace std;

Data data;
Features feature;
// SVM
cv::Mat features_matrix, predictions_vector;
std::vector<int> cellIsPredictable;
std::vector<float> traversability;

cv::Ptr<cv::ml::SVM> svm;
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
		}

	}

	for (int i = 0; i<data.grid_side_length_squared; i++) {
		Point p = grid[i];

		int row = int(p.y*data.px_per_m) + data.height/2;
		int col = int(p.x*data.px_per_m) + data.width/2;

		if (row>=0 && row<data.height && col>=0 && col < data.width) {
			auto & color = img.at<cv::Vec3b>(row, col);
			color[0] = 0;
			color[1] = 0;
			color[2] = 255;

			cv::Rect rect(col-data.sq_px_half, row-data.sq_px_half, data.sq_px, data.sq_px);
//			cv::rectangle(img, rect, cv::Scalar(int(features_matrix.at<float>(i, 11) *1721) % 255,
//                                                int(features_matrix.at<float>(i, 11) *1721) % 255,
//                                                int(features_matrix.at<float>(i, 11)  *173) % 255),
//                                            3);
            if (traversability[i]==0.0f) {
                cv::rectangle(img, rect, cv::Scalar(0, 0, 255),3);
            }
            else if (traversability[i]==1.0f) {
                cv::rectangle(img, rect, cv::Scalar(255, 255, 255),3);
            }
            else if (traversability[i]==2.0f) {
                cv::rectangle(img, rect, cv::Scalar(255, 0, 0),3);
            }
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
	traversability.resize(data.grid_side_length_squared);

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

void predictFeatureMatrix() {
    int64 start = cv::getTickCount();
    svm->predict(features_matrix, predictions_vector);
    auto svm_latency = static_cast<int64>(1000.0f * (float)(cv::getTickCount() - start) / cv::getTickFrequency());
    cout << "SVM_latency: " << svm_latency << endl;

    for (int i = 0; i < predictions_vector.rows; ++i) {
//        cout << cellIsPredictable[i] << " ";
        cout << predictions_vector.at<float>(i, 0) << " ";
    }
    cout << endl;
}

void setPredictedValuesToPredictedCloud() {
    for (int i=0; i<data.grid_side_length_squared; i++) {
        if (cellIsPredictable[i])
            traversability[i] = (predictions_vector.at<float>(i, 0) > 0 ? (float) TRAV_CELL_LABEL : (float) NOT_TRAV_CELL_LABEL);
        else
            traversability[i] =  UNKNOWN_CELL_LABEL;
    }
}



void loadSVM() {
    svm = cv::ml::SVM::load(data.svm_model_path);

    assert(svm->isTrained());

    features_num = svm->getVarCount();

    // stack the indexes of the features that svm will use
    for (int i=0; i<22; ++i) {
        if(columnIsRequested(i, skipColWithIndex))
            feats_indexes.push_back(i);
    }
    loadNormalizationConfig(data.norm_path, features_num, min_vals, p2p_vals);
}

int main() {

	std::vector<Point> grid;
    std::vector<Point> points;
    std::vector< std::vector<Point*> > grid_buckets;
    std::vector<int> labels;


    loadSVM();
	initGrid(grid);

	string lidarpath = data.datapath + "000000.bin";
	string labelpath = data.labelspath + "000000.label";

    // load data from files
	loadLidarScans(lidarpath, points);
    loadLabels(labelpath, labels);


    sortPointsInGrid(points, grid_buckets, data);

    fillFeatureMatrix(points, grid, grid_buckets);

    predictFeatureMatrix();

    setPredictedValuesToPredictedCloud();

    draw(points, labels, grid);


    //projectToImage(points, labels, data);


	return 0;
}
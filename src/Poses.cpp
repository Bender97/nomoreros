//#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
std::vector<cv::Mat> poses;
cv::Mat canvas;
cv::Mat Tr;
cv::Mat Tr_inv;

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

void draw() {

    int width = 1000;
    int height = 1000;

    canvas = cv::Mat::zeros(width, height, CV_8UC3);

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
    float data[16] = {4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, -1.198459927713e-02, -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, -5.403984729748e-02, 9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, -2.921968648686e-01, .0f, .0f ,0.0f, 1.0f};
    Tr = cv::Mat(4, 4, CV_32F, data);
    Tr_inv = Tr.inv();

    loadPoses();

    draw();

    show(canvas, 0);

    return 0;
}
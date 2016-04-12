#include <helper_functions.h>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <cereal/archives/json.hpp>
#include <boost/filesystem.hpp>

using namespace std;

int main(int argc, char** argv)
{
    string data_folder = "/home/nbore/Data/stereo_images";
    string poses_file = data_folder + "/poses.txt";
    string camera1_file = data_folder + "/rgb2_calibrated.txt";
    string camera2_file = data_folder + "/rgb3_calibrated.txt";
    //string camera1_file = data_folder + "/old_calib/rgb2.txt";
    //string camera2_file = data_folder + "/old_calib/rgb3.txt";
    string camera_depth_file = data_folder + "/rgb1_calibrated.txt";
    string param_file = data_folder + "/sgbm_params.json";

    vector<cv::Mat> rgbv1;
    vector<cv::Mat> rgbv2;
    vector<cv::Mat> rgbv3;
    vector<cv::Mat> depthv1;
    bool didparse = parseDataFolder(data_folder, rgbv1, rgbv2, rgbv3, depthv1);
    vector<cv::Mat> disparity_depthv1;
    didparse = parseDataDisparity(data_folder, disparity_depthv1);
    const int N = rgbv1.size();

    cv::Mat residual = cv::Mat::zeros(cv::Size(640, 480), CV_32F);
    cv::Mat counts = cv::Mat::zeros(cv::Size(640, 480), CV_32F);
    for (int i = 0; i < N; ++i) {
        cv::Mat real_depth;
        depthv1[i].convertTo(real_depth, CV_32F, 1./1000);
        real_depth -= disparity_depthv1[i];
        cv::Mat abs_real_depth;
        abs_real_depth = cv::abs(real_depth);
        cv::Mat mask = abs_real_depth < 0.2f;
        cv::bitwise_and(mask, depthv1[i] != 0, mask);
        cv::Mat inverted_mask;
        cv::bitwise_not(mask, inverted_mask);
        real_depth.setTo(cv::Scalar(0), inverted_mask);
        cv::Mat real_mask;
        //cv::cvtColor(mask, real_mask, CV_32F);
        mask.convertTo(real_mask, CV_32F, 1./255);
        counts += real_mask;
        residual += real_depth;
        //cv::Mat real_depth8;
        //cv::normalize(abs_real_depth, real_depth8, 0, 255, CV_MINMAX, CV_8U);
        //cv::imshow("disp", real_depth8);
        //cv::waitKey();
    }

    cv::Mat mask = counts == 0;
    counts.setTo(cv::Scalar(1.0f), mask);
    residual /= counts;

    cv::Mat mask_certainty = counts < 3;
    residual.setTo(cv::Scalar(0.0f), mask_certainty);

    cv::Mat residual8;
    cv::normalize(residual, residual8, 0, 255, CV_MINMAX, CV_8U);
    cv::imshow("residual", residual8);
    cv::waitKey();

    return 0;
}

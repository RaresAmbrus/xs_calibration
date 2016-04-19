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
#include <sparse_gp/sparse_gp.h>
#include <sparse_gp/gaussian_noise.h>
#include <sparse_gp/rbf_kernel_3d.h>

using namespace std;

pair<cv::Mat, cv::Mat> compute_residual(cv::Mat& depth, cv::Mat& disparity_depth)
{
    cv::Mat residual = depth - disparity_depth;
    cv::Mat abs_residual = cv::abs(residual);
    cv::Mat mask = abs_residual < 0.1f;
    cv::bitwise_and(mask, depth != 0, mask);
    cv::Mat inverted_mask;
    cv::bitwise_not(mask, inverted_mask);
    residual.setTo(cv::Scalar(0), inverted_mask);
    cv::Mat counts;
    mask.convertTo(counts, CV_32F, 1./255);
    return make_pair(residual, counts);
}

void normalize_residuals(cv::Mat& residual, cv::Mat& counts)
{
    cv::Mat mask = counts == 0;
    counts.setTo(cv::Scalar(1.0f), mask);
    residual /= counts;

    cv::Mat mask_certainty = counts < 3;
    residual.setTo(cv::Scalar(0.0f), mask_certainty);
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        cout << "Please provide the data folder..." << endl;
        return 0;
    }
    string data_folder(argv[1]); // = "/home/nbore/Data/stereo_images";
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

    // now, let's set up residuals and counts for different depth levels
    // importantly, the closer the level, the smaller the considered residuals
    // 0-1, 1-2, 2-3, 3-4, 4-5, 5-6, 6-7
    const int nbr_levels = 7;
    vector<cv::Mat> level_residuals(nbr_levels);
    vector<cv::Mat> level_counts(nbr_levels);
    for (int level = 0; level < nbr_levels; ++level) {
        level_residuals[level] = cv::Mat::zeros(cv::Size(640, 480), CV_32F);
        level_counts[level] = cv::Mat::zeros(cv::Size(640, 480), CV_32F);
    }

    sparse_gp<rbf_kernel_3d, gaussian_noise> gp;
    for (int i = 0; i < N; ++i) {
        cv::Mat real_depth;
        depthv1[i].convertTo(real_depth, CV_32F, 1./1000);

        /*
        cv::Mat scan_residual = real_depth - disparity_depthv1[i];
        cv::Mat abs_residual = cv::abs(scan_residual);
        cv::Mat mask = abs_residual < 0.1f;
        cv::bitwise_and(mask, depthv1[i] != 0, mask);
        cv::Mat inverted_mask;
        cv::bitwise_not(mask, inverted_mask);
        scan_residual.setTo(cv::Scalar(0), inverted_mask);
        cv::Mat real_mask;
        //cv::cvtColor(mask, real_mask, CV_32F);
        mask.convertTo(real_mask, CV_32F, 1./255);
        counts += real_mask;
        residual += scan_residual;
        */
        cv::Mat scan_residual, scan_counts;
        tie(scan_residual, scan_counts) = compute_residual(real_depth, disparity_depthv1[i]);
        residual += scan_residual;
        counts += scan_counts;

        // X = x pixel, y pixel, depth in original image
        // y = residual
        Eigen::MatrixXd X;
        Eigen::VectorXd y;
        gp.add_measurements(X, y);

        for (int level = 0; level < nbr_levels; ++level) {
            cv::Mat depth_mask = real_depth < float(level+1);
            cv::bitwise_and(depth_mask, real_depth > float(level), depth_mask);
            cv::Mat level_depth = cv::Mat::zeros(cv::Size(640, 480), CV_32F);
            real_depth.copyTo(level_depth, depth_mask);

            cv::Mat level_residual, level_count;
            tie(level_residual, level_count) = compute_residual(level_depth, disparity_depthv1[i]);
            level_residuals[level] += level_residual;
            level_counts[level] += level_count;
        }

        //cv::Mat real_depth8;
        //cv::normalize(abs_real_depth, real_depth8, 0, 255, CV_MINMAX, CV_8U);
        //cv::imshow("disp", real_depth8);
        //cv::waitKey();
    }

    /*
    cv::Mat mask = counts == 0;
    counts.setTo(cv::Scalar(1.0f), mask);
    residual /= counts;

    cv::Mat mask_certainty = counts < 3;
    residual.setTo(cv::Scalar(0.0f), mask_certainty);
    */
    normalize_residuals(residual, counts);

    cv::Mat residual8;
    cv::normalize(residual, residual8, 0, 255, CV_MINMAX, CV_8U);
    cv::imshow("residual", residual8);
    cv::waitKey();

    cv::imwrite(data_folder + "/residual.png", residual8);

    for (int level = 0; level < nbr_levels; ++level) {

        // X = x pixel, y pixel, depth in original image
        // f_star = estimated residual
        // V_star = estimated variance
        Eigen::MatrixXd X_star;
        Eigen::VectorXd f_star;
        Eigen::VectorXd V_star;
        gp.predict_measurements(f_star, X_star, V_star);

        normalize_residuals(level_residuals[level], level_counts[level]);
        cv::Mat residual8;
        cv::normalize(level_residuals[level], residual8, 0, 255, CV_MINMAX, CV_8U);
        cv::imshow("residual", residual8);
        cv::waitKey();

        cv::imwrite(data_folder + "/residual_level" + to_string(level) + ".png", residual8);
    }

    return 0;
}

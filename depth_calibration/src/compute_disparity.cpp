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

int pair_number;

Eigen::Matrix4f read_transform(ifstream& in)
{
    Eigen::Matrix4f transform;
    float temp;
    for (size_t j=0; j<4; j++){
        for (size_t k=0; k<4; k++){
            in >> temp;
            transform(j, k) = temp;
        }
    }
    return transform;
}

pair<Eigen::Matrix3f, Eigen::Matrix<float, 5, 1> > read_camera(const string& camera_file)
{
    ifstream in(camera_file);
    Eigen::Matrix3f camera;
    Eigen::Matrix<float, 5, 1> distortion;
    camera.setIdentity();
    distortion.setZero();
    in >> camera(0, 0);
    in >> camera(1, 1);
    in >> camera(0, 2);
    in >> camera(1, 2);
    in >> distortion(0);
    in >> distortion(1);
    in.close();

    distortion(0) = 0*distortion(0);
    distortion(1) = 0*distortion(1);

    return make_pair(camera, distortion);
}

tuple<cv::Mat, cv::Mat, cv::Mat> rectify_images(cv::Mat& rgb1, const Eigen::Matrix3f& K1, const Eigen::Matrix<float, 5, 1>& D1, const Eigen::Matrix4f& T1,
                                                cv::Mat& rgb2, const Eigen::Matrix3f& K2, const Eigen::Matrix<float, 5, 1>& D2, const Eigen::Matrix4f& T2,
                                                int rectify_mode)
{
    cv::Mat img1;
    cv::Mat img2;
    cv::cvtColor(rgb1, img1, CV_BGR2GRAY);
    cv::cvtColor(rgb2, img2, CV_BGR2GRAY);

    cv::Mat_<double> cameraMatrix1(3,3); // 3x3 matrix
    cv::Mat_<double> distCoeffs1(5,1);   // 5x1 matrix for five distortion coefficients
    cv::Mat_<double> cameraMatrix2(3,3); // 3x3 matrix
    cv::Mat_<double> distCoeffs2(5,1);   // 5x1 matrix

    cameraMatrix1 << K1(0, 0), K1(0, 1), K1(0, 2), K1(1, 0), K1(1, 1), K1(1, 2), K1(2, 0), K1(2, 1), K1(2, 2);
    cameraMatrix2 << K2(0, 0), K2(0, 1), K2(0, 2), K2(1, 0), K2(1, 1), K2(1, 2), K2(2, 0), K2(2, 1), K2(2, 2);
    //distCoeffs1 << D1(0), D1(1), D1(2), D1(3), D1(4);
    //distCoeffs2 << D2(0), D2(1), D2(2), D2(3), D2(4);
    //distCoeffs1 << 0.053815, -0.241557, 0, 0, 0;
    //distCoeffs2 << 0.053815, -0.241557, 0, 0, 0;
    //distCoeffs2 << 0.254065 -1.20616, 0, 0, 0;
    distCoeffs1 << 0.155566594526053, -0.4125181976686453, -0.002011408581651128, -0.003590837765389968, 0.0;
    distCoeffs2 << 0.155566594526053, -0.4125181976686453, -0.002011408581651128, -0.003590837765389968, 0.0;

    Eigen::Affine3f A1(T1);
    Eigen::Affine3f A2(T2);
    Eigen::Affine3f A12;
    if (rectify_mode == 0) {
        A12 = A2*A1.inverse();
    }
    else if (rectify_mode == 1) {
        A12 = A1.inverse()*A2;
    }
    else if (rectify_mode == 2) {
        A12 = A1*A2.inverse();
    }
    else {
        A12 = A2.inverse()*A1;
    }
    Eigen::Vector3f trans = A12.translation();
    Eigen::Matrix3f rot = A12.rotation();

    cv::Mat_<double> R(3, 3);
    cv::Mat_<double> T(3, 1);
    R << rot(0, 0), rot(0, 1), rot(0, 2), rot(1, 0), rot(1, 1), rot(1, 2), rot(2, 0), rot(2, 1), rot(2, 2);
    T << trans(0), trans(1), trans(2);

    cv::Mat R1; // 3x3 matrix
    cv::Mat R2; // 3x3 matrix
    cv::Mat P1; // 3x4 matrix
    cv::Mat P2; // 3x4 matrix
    cv::Mat Q;  // 4x4 matrix

    cv::Rect valid_left;
    cv::Rect valid_right;
    cv::stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, img1.size(), R, T, R1, R2, P1, P2, Q, 0, -1, cv::Size(), &valid_left, &valid_right);

    cv::Mat lMap1;
    cv::Mat lMap2;
    cv::Mat rMap1;
    cv::Mat rMap2;

    cv::initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, img1.size(), CV_8UC1, lMap1, lMap2);
    cv::initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, img1.size(), CV_8UC1, rMap1, rMap2);

    cv::Mat leftImageMatO;
    cv::Mat rightImageMatO;

    cv::remap(img1, leftImageMatO, lMap1, lMap2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::remap(img2, rightImageMatO, rMap1, rMap2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    //leftImageMatO = leftImageMatO(valid_left);
    //rightImageMatO = rightImageMatO(valid_left);

    return make_tuple(leftImageMatO, rightImageMatO, Q);
}

struct sgbm_params {
    int SADWindowSize;
    int numberOfDisparities;
    int preFilterCap;
    int minDisparity;
    int uniquenessRatio;
    int speckleWindowSize;
    int speckleRange;
    int disp12MaxDiff;
    int P1;
    int P2;

    sgbm_params() {
        SADWindowSize = 10;
        numberOfDisparities = 9;
        preFilterCap = 63;
        minDisparity = -72;
        uniquenessRatio = 5;
        speckleWindowSize = 200;
        speckleRange = 32;
        disp12MaxDiff = -1;
        P1 = 8*1*SADWindowSize*SADWindowSize;
        P2 = 32*1*SADWindowSize*SADWindowSize;
    }

    template <class Archive>
    void serialize(Archive& archive)
    {
        archive(cereal::make_nvp("SADWindowSize", SADWindowSize),
                cereal::make_nvp("numberOfDisparities", numberOfDisparities),
                cereal::make_nvp("preFilterCap", preFilterCap),
                cereal::make_nvp("minDisparity", minDisparity),
                cereal::make_nvp("uniquenessRatio", uniquenessRatio),
                cereal::make_nvp("speckleWindowSize", speckleWindowSize),
                cereal::make_nvp("speckleRange", speckleRange),
                cereal::make_nvp("disp12MaxDiff", disp12MaxDiff));
        P1 = 8*1*SADWindowSize*SADWindowSize;
        P2 = 32*1*SADWindowSize*SADWindowSize;
    }
};

cv::Mat compute_disparity(cv::Mat& g1, cv::Mat& g2, const sgbm_params& params)
{
    cv::Mat disp;

    cv::StereoSGBM sbm;
    sbm.SADWindowSize = params.SADWindowSize;
    sbm.numberOfDisparities = 16*params.numberOfDisparities;
    sbm.preFilterCap = params.preFilterCap;
    sbm.minDisparity = params.minDisparity;
    sbm.uniquenessRatio = params.uniquenessRatio;
    sbm.speckleWindowSize = params.speckleWindowSize;
    sbm.speckleRange = params.speckleRange;
    sbm.disp12MaxDiff = params.disp12MaxDiff;
    sbm.fullDP = false;
    //sbm.P1 = 8*1*sbm.SADWindowSize*sbm.SADWindowSize;
    //sbm.P2 = 32*1*sbm.SADWindowSize*sbm.SADWindowSize;
    sbm.P1 = params.P1;
    sbm.P2 = params.P2;
    sbm(g1, g2, disp);

    cv::Mat disp32;
    disp.convertTo(disp32, CV_32F, 1./16);

    return disp32;
}

// this should take in the transform of the left camera I guess? We know the Kinect is at (0, 0, 0)
cv::Mat depth_image_from_disparity(cv::Mat& disparity, cv::Mat& Q, const Eigen::Matrix3f& K, const Eigen::Matrix<float, 5, 1>& D,
                                   const Eigen::Matrix4f& left_camera_transform)
{
    cv::Mat image3d;
    cv::reprojectImageTo3D(disparity, image3d, Q, false, -1);

    std::vector<cv::Point3f> array;
    array.assign((cv::Point3f*)image3d.datastart, (cv::Point3f*)image3d.dataend);

    Eigen::Affine3f A(left_camera_transform);
    A = A.inverse();
    Eigen::Vector3f trans = A.translation();
    Eigen::Matrix3f rot = A.rotation();

    // transform of depth camera in left camera?
    cv::Mat_<double> R(3, 3);
    cv::Mat_<double> T(3, 1);
    R << rot(0, 0), rot(0, 1), rot(0, 2), rot(1, 0), rot(1, 1), rot(1, 2), rot(2, 0), rot(2, 1), rot(2, 2);
    T << trans(0), trans(1), trans(2);
    // camera parameters of depth rgb camera
    cv::Mat_<double> cameraMatrix(3,3); // 3x3 matrix
    cameraMatrix << K(0, 0), K(0, 1), K(0, 2), K(1, 0), K(1, 1), K(1, 2), K(2, 0), K(2, 1), K(2, 2);
    cv::Mat_<double> distCoeffs(5,1);   // 5x1 matrix for five distortion coefficients
    distCoeffs << 0, 0, 0, 0, 0; // the depth camera is assumed to have 0 distortion

    vector<cv::Point2f> image_points(array.size());
    //cv::Mat image_points;
    cv::projectPoints(array, R, T, cameraMatrix, distCoeffs, image_points, cv::noArray(), 0);
    // image points now simply contain the 2d points corresponding to the depths below
    // so, we can just iterate through an image and assign the pixels

    cv::Mat channels[3];
    cv::split(image3d, channels);
    cv::Mat depth = channels[2];
    //cout << depth << endl;
    //cv::Mat mask = depth > 2;
    //depth.setTo(cv::Scalar(0.0f), mask);

    cv::Mat reprojected_depth = cv::Mat::zeros(480, 640, CV_32F);
    for (int i = 0; i < image_points.size(); ++i) {
        int x = int(image_points[i].x);
        int y = int(image_points[i].y);
        if (x < 0 || x >= 640 || y < 0 || y >= 480) {
            continue;
        }
        if (array[i].z < 7) {
            reprojected_depth.at<float>(y, x) = array[i].z;
        }
    }

    return reprojected_depth;
}

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

    ifstream in(poses_file);
    Eigen::Matrix4f transform1 = read_transform(in);
    Eigen::Matrix4f transform2 = read_transform(in);
    in.close();

    Eigen::Matrix3f camera1;
    Eigen::Matrix<float, 5, 1> distortion1;
    tie(camera1, distortion1) = read_camera(camera1_file);
    Eigen::Matrix3f camera2;
    Eigen::Matrix<float, 5, 1> distortion2;
    tie(camera2, distortion2) = read_camera(camera2_file);
    Eigen::Matrix3f camera_depth;
    Eigen::Matrix<float, 5, 1> distortion_depth;
    tie(camera_depth, distortion_depth) = read_camera(camera_depth_file);

    cout << transform1 << endl;
    cout << transform2 << endl;

    vector<cv::Mat> rgbv1;
    vector<cv::Mat> rgbv2;
    vector<cv::Mat> rgbv3;
    vector<cv::Mat> depthv1;
    bool didparse = parseDataFolder(data_folder, rgbv1, rgbv2, rgbv3, depthv1);

    const int N = rgbv1.size();

    // Create a window
    cv::namedWindow("disp", cv::WINDOW_NORMAL);

    sgbm_params params;
    if (boost::filesystem::exists(param_file)) {
        std::ifstream in(param_file);
        {
            cereal::JSONInputArchive archive_i(in);
            archive_i(params);
        }
    }
    params.minDisparity += 100;

    cv::createTrackbar("SADWindowSize", "disp", &params.SADWindowSize, 100);
    cv::createTrackbar("numberOfDisparities", "disp", &params.numberOfDisparities, 30);
    cv::createTrackbar("preFilterCap", "disp", &params.preFilterCap, 100);
    cv::createTrackbar("minDisparity", "disp", &params.minDisparity, 200);
    cv::createTrackbar("uniquenessRatio", "disp", &params.uniquenessRatio, 20);
    cv::createTrackbar("speckleWindowSize", "disp", &params.speckleWindowSize, 2000);
    cv::createTrackbar("speckleRange", "disp", &params.speckleRange, 100);
    cv::createTrackbar("disp12MaxDiff", "disp", &params.disp12MaxDiff, 10);
    cv::createTrackbar("P1", "disp", &params.P1, 6000);
    cv::createTrackbar("P2", "disp", &params.P2, 6000);
    int rectify_mode = 1;
    cv::createTrackbar("rectifyMode", "disp", &rectify_mode, 3);
    int prev_rectify_mode = rectify_mode;

    string image_folder = data_folder + "/rectified";
    for (int i = 0; i < N; ++i) {
        cv::Mat rectified1;
        cv::Mat rectified2;
        cv::Mat Q;
        tie(rectified1, rectified2, Q) = rectify_images(rgbv2[i], camera1, distortion1, transform1,
                                                        rgbv3[i], camera2, distortion2, transform2, rectify_mode);
        cv::imwrite(image_folder + "/left" + to_string(i) + ".png", rectified1);
        cv::imwrite(image_folder + "/right" + to_string(i) + ".png", rectified2);
        /*cv::namedWindow("rectified1", cv::WINDOW_NORMAL);
        cv::imshow("rectified1", rectified1);
        cv::namedWindow("rectified2", cv::WINDOW_NORMAL);
        cv::imshow("rectified2", rectified2);
        cv::waitKey();*/

        while (true) {
            if (prev_rectify_mode != rectify_mode) {
                tie(rectified1, rectified2, Q) = rectify_images(rgbv2[i], camera1, distortion1, transform1,
                                                                rgbv3[i], camera2, distortion2, transform2, rectify_mode);
                prev_rectify_mode = rectify_mode;
            }
            params.minDisparity -= 100;
            cv::Mat disp = compute_disparity(rectified1, rectified2, params);
            params.minDisparity += 100;
            Eigen::Matrix4f T;
            T.setIdentity();
            cv::Mat depth = depth_image_from_disparity(disp, Q, camera_depth, distortion_depth, T);
            cv::Mat disp8;
            double min, max;
            cv::minMaxLoc(depth, &min, &max);
            cout << "Min: " << min << endl;
            cout << "Max: " << max << endl;
            cv::normalize(depth, disp8, 0, 255, CV_MINMAX, CV_8U);
            cv::Mat depth8;
            cv::normalize(depthv1[i], depth8, 0, 255, CV_MINMAX, CV_8U);

            cv::Mat real_depth;
            depthv1[i].convertTo(real_depth, CV_32F, 1./1000);
            real_depth -= depth;
            real_depth = cv::abs(real_depth);
            cv::Mat real_depth8;
            cv::normalize(real_depth, real_depth8, 0, 255, CV_MINMAX, CV_8U);

            //cv::Mat real_depth;
            //cv::cvtColor(depthv1[i], real_depth, CV_32F);

            vector<cv::Mat> channels;
            cv::Mat Z = cv::Mat::zeros(cv::Size(640, 480), CV_8U);
            channels.push_back(Z);
            channels.push_back(depth8);
            channels.push_back(disp8);
            cv::Mat vis_img;
            cv::merge(channels, vis_img);

            cv::Mat disp_img(cv::Size(1920, 480), CV_8UC3);
            cv::Mat left(disp_img, cv::Rect(0, 0, 640, 480)); // Copy constructor
            vis_img.copyTo(left);
            cv::Mat right(disp_img, cv::Rect(640, 0, 640, 480)); // Copy constructor
            //imgorig2.copyTo(right);
            cv::cvtColor(disp8, right, CV_GRAY2BGR);
            cv::Mat far(disp_img, cv::Rect(1280, 0, 640, 480));
            cv::cvtColor(real_depth8, far, CV_GRAY2BGR);

            cv::imshow("disp", disp_img);

            // Wait until user press some key for 50ms
            char key = cv::waitKey();

            //if user press 'ESC' key
            if (key == 'n') {
                break;
            }
            else if (key == 'p') {
                i -= 2;
                break;
            }
            else if (key == 's') {
                params.minDisparity -= 100;
                std::ofstream out(param_file);
                {
                    cereal::JSONOutputArchive archive_o(out);
                    archive_o(params);
                }
                params.minDisparity += 100;
            }
        }
    }

    return 0;
}

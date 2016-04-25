#include "helper_functions.h"
#include <QDir>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>


bool parseDataFolder(std::string data_folder, std::vector<cv::Mat>& rgbv1, std::vector<cv::Mat>& rgbv2, std::vector<cv::Mat>& rgbv3, std::vector<cv::Mat>& depthv1){
    using namespace std;

    data_folder+="/";
    cout<<"Looking for XS images in "<<data_folder<<endl;

    bool files_found = true;

    int image_counter = 0;
    while (files_found){
        stringstream rgb1_file_ss; rgb1_file_ss<<"rgb_1_"<<std::setfill('0')<<std::setw(4)<<image_counter<<".jpg";
        string rgb1_file = data_folder+rgb1_file_ss.str();
        if (!QFile(rgb1_file.c_str()).exists()){
            files_found = false;
            break;
        }

        stringstream rgb2_file_ss; rgb2_file_ss<<"rgb_2_"<<std::setfill('0')<<std::setw(4)<<image_counter<<".jpg";
        string rgb2_file = data_folder+rgb2_file_ss.str();
        if (!QFile(rgb2_file.c_str()).exists()){
            files_found = false;
            break;
        }

        stringstream rgb3_file_ss; rgb3_file_ss<<"rgb_3_"<<std::setfill('0')<<std::setw(4)<<image_counter<<".jpg";
        string rgb3_file = data_folder+rgb3_file_ss.str();
        if (!QFile(rgb3_file.c_str()).exists()){
            files_found = false;
            break;
        }

        stringstream depth1_file_ss; depth1_file_ss<<"depth_1_"<<std::setfill('0')<<std::setw(4)<<image_counter<<".png";
        string depth1_file = data_folder+depth1_file_ss.str();
        if (!QFile(depth1_file.c_str()).exists()){
            files_found = false;
            break;
        }

        cv::Mat rgb1 = cv::imread(rgb1_file.c_str());
        rgbv1.push_back(rgb1);
        cv::Mat rgb2 = cv::imread(rgb2_file.c_str());
        rgbv2.push_back(rgb2);
        cv::Mat rgb3 = cv::imread(rgb3_file.c_str());
        rgbv3.push_back(rgb3);
        cv::Mat depth1 = cv::imread(depth1_file.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
        depthv1.push_back(depth1);

        image_counter++;
    }

    if (rgbv1.size()){
        cout<<"Found "<<rgbv1.size()<<" images "<<endl;
        return true;
    } else {
        return false;
    }
}

bool parseDataDisparity(std::string data_folder, std::vector<cv::Mat>& depthv1)
{
    using namespace std;

    data_folder+="/";
    cout<<"Looking for XS images in "<<data_folder<<endl;

    bool files_found = true;

    int image_counter = 0;
    while (files_found){
        stringstream depth1_file_ss; depth1_file_ss<<"disparity_depth_1_"<<std::setfill('0')<<std::setw(4)<<image_counter<<".yml";
        string depth1_file = data_folder+depth1_file_ss.str();
        if (!QFile(depth1_file.c_str()).exists()){
            files_found = false;
            break;
        }

        cv::Mat depth1;// = cv::imread(depth1_file.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
        cv::FileStorage fs(depth1_file, cv::FileStorage::READ );
        fs["mat1"] >> depth1;
        depthv1.push_back(depth1);

        image_counter++;
    }

    if (depthv1.size()){
        cout<<"Found "<<depthv1.size()<<" images "<<endl;
        return true;
    } else {
        return false;
    }
}

bool parseDataResiduals(std::string data_folder, std::vector<cv::Mat>& residuals)
{
    using namespace std;

    data_folder+="/";
    cout<<"Looking for XS images in "<<data_folder<<endl;

    bool files_found = true;

    int image_counter = 0;
    while (files_found){
        stringstream residual_file_ss; residual_file_ss<<"/residual_level_"<<std::setfill('0')<<std::setw(4)<<image_counter<<".yml";
        string residual_file = data_folder+residual_file_ss.str();
        if (!QFile(residual_file.c_str()).exists()){
            files_found = false;
            break;
        }

        cv::Mat residual;// = cv::imread(depth1_file.c_str(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR);
        cv::FileStorage fs(residual_file, cv::FileStorage::READ );
        fs["mat1"] >> residual;
        residuals.push_back(residual);

        image_counter++;
    }

    if (residuals.size()){
        cout<<"Found "<<residuals.size()<<" images "<<endl;
        return true;
    } else {
        return false;
    }
}

void visualize_data(const std::vector<cv::Mat>& rgbv1, const std::vector<cv::Mat>& rgbv2,
                    const std::vector<cv::Mat>& rgbv3, const std::vector<cv::Mat>& depthv1){
    cv::namedWindow("test", cv::WINDOW_OPENGL);
    for (size_t i=0; i<rgbv1.size();i++){
        cv::imshow("test",rgbv1[i]);
        cv::waitKey(0);
        cv::imshow("test",rgbv2[i]);
        cv::waitKey(0);
        cv::imshow("test",rgbv3[i]);
        cv::waitKey(0);
        cv::imshow("test",depthv1[i]);
        cv::waitKey(0);
    }
}

void initializeCamParamsFromFiles(const std::string& folder, double* params_rgb1, double* params_rgb2, double* params_rgb3){
    using namespace std;


    string params_file1, params_file2, params_file3;
    params_file1 = folder + "/rgb1_calibrated.txt"; // xtion
    params_file2 = folder + "/rgb2_calibrated.txt"; // XS1
    params_file3 = folder + "/rgb3_calibrated.txt"; // XS2

    ifstream in1, in2, in3;
    in1.open(params_file1.c_str());
    in1>>params_rgb1[0]>>params_rgb1[1]>>params_rgb1[2]>>params_rgb1[3]>>params_rgb1[4]>>params_rgb1[5];
    in1.close();
    cout<<"Loading camera parameters for camera 1 from "<<params_file1<<endl;

    in2.open(params_file2.c_str());
    in2>>params_rgb2[0]>>params_rgb2[1]>>params_rgb2[2]>>params_rgb2[3]>>params_rgb2[4]>>params_rgb2[5];
    in2.close();
    cout<<"Loading camera parameters for camera 2 from "<<params_file2<<endl;

    in3.open(params_file3.c_str());
    in3>>params_rgb3[0]>>params_rgb3[1]>>params_rgb3[2]>>params_rgb3[3]>>params_rgb3[4]>>params_rgb3[5];
    in3.close();
    cout<<"Loading camera parameters for camera 3 from "<<params_file3<<endl;

    return;
}

std::vector<Eigen::Matrix4f> loadPosesFromFile(const std::string& poses_file, int no_transforms){

    std::vector<Eigen::Matrix4f> toRet;

    std::ifstream in;
    in.open(poses_file);

    if (!in.is_open()){
        std::cout<<"Could not open file to read poses. Aborting."<<std::endl;
        return toRet;
    }

    for (size_t i=0; i<no_transforms; i++){
        Eigen::Matrix4f transform;
        float temp;
        for (size_t j=0; j<4; j++){
            for (size_t k=0; k<4; k++){
                in >> temp;
                transform(j,k) = temp;
            }
        }
        toRet.push_back(transform);
    }
}

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> createCloudFromImages(cv::Mat rgb, cv::Mat depth, const double* params)
{
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    double cx = 0.001 / params[0];
    double cy = 0.001 / params[1];
    double center_x = params[2];
    double center_y = params[3];

    for (int v = 0; v < rgb.rows; ++v)
    {
        for (int u = 0; u < rgb.cols; ++u)
        {
            pcl::PointXYZRGB pt;
            const float point_depth = (float)depth.at<u_int16_t>(v,u);

            if (!(point_depth != 0))
            {
                pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
            } else {
                // Fill in XYZ
                pt.x = (u - center_x) * point_depth * cx;
                pt.y = (v - center_y) * point_depth * cy;
                pt.z = (float) point_depth * 0.001f; // convert to uint16 to meters
            }

            uint32_t point_rgb = ((uint8_t)rgb.at<cv::Vec3b>(v, u)[2] << 16 | (uint8_t) rgb.at<cv::Vec3b>(v, u)[1] << 8 | (uint8_t)rgb.at<cv::Vec3b>(v, u)[0]);
            pt.rgb = *reinterpret_cast<float*>(&point_rgb);
            cloud->push_back(pt);
        }
    }

    return cloud;
}

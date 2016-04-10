#include "helper_functions.h"
#include <QDir>
#include <iostream>
#include <sstream>
#include <iomanip>


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

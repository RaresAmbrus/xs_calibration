#ifndef __XS_CALIBRATION__HELPER__
#define __XS_CALIBRATION__HELPER__

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>


bool parseDataFolder(std::string data_folder, std::vector<cv::Mat>& rgbv1, std::vector<cv::Mat>& rgbv2, std::vector<cv::Mat>& rgbv3, std::vector<cv::Mat>& depthv1);


#endif

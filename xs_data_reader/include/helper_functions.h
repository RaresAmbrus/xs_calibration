#ifndef __XS_CALIBRATION__HELPER__
#define __XS_CALIBRATION__HELPER__

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>

bool parseDataFolder(std::string data_folder, std::vector<cv::Mat>& rgbv1, std::vector<cv::Mat>& rgbv2, std::vector<cv::Mat>& rgbv3, std::vector<cv::Mat>& depthv1);

void visualize_data(const std::vector<cv::Mat>& rgbv1, const std::vector<cv::Mat>& rgbv2,
                    const std::vector<cv::Mat>& rgbv3, const std::vector<cv::Mat>& depthv1);

void initializeCamParamsFromFiles(const std::string& folder, double* params_rgb1, double* params_rgb2, double* params_rgb3);

std::vector<Eigen::Matrix4f> loadPosesFromFile(const std::string& poses_file, int no_transforms);

#endif

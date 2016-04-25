#include <glog/logging.h>
#include "helper_functions.h"
#include <tf_conversions/tf_eigen.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std;


int main(int argc, char** argv)
{
    string folder;

    if (argc > 1){
        folder = argv[1];
    } else {
        cout<<"Please pass the data folder as argument. Exitting."<<endl;
        return -1;
    }

    // get input data
    std::vector<cv::Mat> rgbv1, rgbv2, rgbv3, depthv1;
    parseDataFolder(folder, rgbv1, rgbv2, rgbv3, depthv1);

    // initialize camera parameters
    double params_rgb1[6], params_rgb2[6], params_rgb3[6];
    initializeCamParamsFromFiles(folder,params_rgb1, params_rgb2, params_rgb3);


    ros::init(argc, argv, "xs_publisher");
    ros::NodeHandle n("~");

    boost::shared_ptr<image_transport::ImageTransport> debug_img_it_;
    image_transport::Publisher debug_img_pub_;
    image_transport::Publisher debug_img_pub_2;
    debug_img_it_.reset(new image_transport::ImageTransport(n));
    debug_img_pub_ = debug_img_it_->advertise("/xs1/image", 1);
    debug_img_pub_2 = debug_img_it_->advertise("/xs2/image", 1);

    // cam info
    ros::Publisher xs1CamInfo = n.advertise<sensor_msgs::CameraInfo>("/xs1/camera_info", 1000);
    ros::Publisher xs2CamInfo = n.advertise<sensor_msgs::CameraInfo>("/xs2/camera_info", 1000);
    ros::Publisher xtionCamInfo = n.advertise<sensor_msgs::CameraInfo>("/xtion/camera_info", 1000);

    // cloud
     ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("/xtion_cloud", 1);

    sensor_msgs::CameraInfo xs1_cam_info;
    xs1_cam_info.K[0] = params_rgb2[0];
    xs1_cam_info.K[4] = params_rgb2[1];
    xs1_cam_info.K[2] = params_rgb2[2];
    xs1_cam_info.K[5] = params_rgb2[3];
    xs1_cam_info.K[8] = 1.0;
    xs1_cam_info.P[0] = params_rgb2[0];
    xs1_cam_info.P[2] = params_rgb2[2];
    xs1_cam_info.P[5] = params_rgb2[1];
    xs1_cam_info.P[6] = params_rgb2[3];
    xs1_cam_info.P[10] = 1.0;
//    xs1_cam_info.D.push_back(params_rgb2[4]);
//    xs1_cam_info.D.push_back(params_rgb2[5]);
    xs1_cam_info.D = {params_rgb2[4], params_rgb2[5], 0.0, 0.0, 0.0};
//    xs1_cam_info.D = {0.155566594526053, -0.4125181976686453, -0.002011408581651128, -0.003590837765389968, 0.0}; // original distortion parameters
    xs1_cam_info.header.frame_id="/XS1";

    sensor_msgs::CameraInfo xs2_cam_info;
    xs2_cam_info.K[0] = params_rgb3[0];
    xs2_cam_info.K[4] = params_rgb3[1];
    xs2_cam_info.K[2] = params_rgb3[2];
    xs2_cam_info.K[5] = params_rgb3[3];
    xs2_cam_info.K[8] = 1.0;
    xs2_cam_info.P[0] = params_rgb3[0];
    xs2_cam_info.P[2] = params_rgb3[2];
    xs2_cam_info.P[5] = params_rgb3[1];
    xs2_cam_info.P[6] = params_rgb3[3];
    xs2_cam_info.P[10] = 1.0;
//    xs2_cam_info.D.push_back(params_rgb3[4]);
//    xs2_cam_info.D.push_back(params_rgb3[5]);
    xs2_cam_info.D = {params_rgb3[4], params_rgb3[5], 0.0, 0.0, 0.0};
//    xs2_cam_info.D = {0.155566594526053, -0.4125181976686453, -0.002011408581651128, -0.003590837765389968, 0.0}; // original distortion parameters
    xs2_cam_info.header.frame_id="/XS2";



    ros::Rate rate(0.2f);
    int current_image = 0;

    std_msgs::Header img_header_xs1, img_header_xs2;
    img_header_xs1.frame_id = "/XS1";
    img_header_xs2.frame_id = "/XS2";

    while (n.ok()) {

        cout<<"Publishing image "<<current_image<<endl;

        debug_img_pub_.publish(
            cv_bridge::CvImage(img_header_xs1, "bgr8", rgbv2[current_image]).toImageMsg());
        xs1CamInfo.publish(xs1_cam_info);

        debug_img_pub_2.publish(
            cv_bridge::CvImage(img_header_xs2, "bgr8", rgbv3[current_image]).toImageMsg());
        xs2CamInfo.publish(xs2_cam_info);

        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud = createCloudFromImages(rgbv1[current_image], depthv1[current_image], params_rgb1);
        sensor_msgs::PointCloud2 msg_cloud;
        pcl::toROSMsg(*cloud, msg_cloud);
        msg_cloud.header.frame_id = "xtion";
        cloud_pub.publish(msg_cloud);

        if (current_image == rgbv2.size()-1){
            current_image = -1;
        }
        current_image++;

        rate.sleep();
    }
}

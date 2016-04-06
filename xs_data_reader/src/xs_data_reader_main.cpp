#include <glog/logging.h>
#include "helper_functions.h"

#include <pcl/visualization/pcl_visualizer.h>
#include <tf_conversions/tf_eigen.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

using namespace std;

void testTransformsROS(int argc, char** argv, const std::vector<tf::Transform>& transforms);


int main(int argc, char** argv)
{
    string folder;

    cout<<"Argc is "<<argc<<endl;

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
    initializeCamParamsFromFiles(folder, params_rgb1, params_rgb2, params_rgb3);
    cout<<"Xtion camera parameters: "<<params_rgb1[0]<<" "<<params_rgb1[1]<<" "<<params_rgb1[2]<<" "<<params_rgb1[3]<<" "<<params_rgb1[4]<<" "<<params_rgb1[5]<<endl;
    cout<<"XS1 camera parameters: "<<params_rgb2[0]<<" "<<params_rgb2[1]<<" "<<params_rgb2[2]<<" "<<params_rgb2[3]<<" "<<params_rgb2[4]<<" "<<params_rgb2[5]<<endl;
    cout<<"XS2 camera parameters: "<<params_rgb3[0]<<" "<<params_rgb3[1]<<" "<<params_rgb3[2]<<" "<<params_rgb3[3]<<" "<<params_rgb3[4]<<" "<<params_rgb3[5]<<endl;

    // load camera transforms
    std::vector<Eigen::Matrix4f> camera_transforms = loadPosesFromFile(folder + "/poses.txt",2);
    cout<<"Transform Xtion -> XS1 "<<endl<<camera_transforms[0]<<endl<<endl;
    cout<<"Transform Xtion -> XS2 "<<endl<<camera_transforms[1]<<endl<<endl;

    // convert transforms to tf
    vector<tf::Transform> camera_transforms_tf;
    for (auto camera : camera_transforms){
        tf::Transform camera_tf;
        tf::transformEigenToTF(Eigen::Affine3d(camera.cast<double>()), camera_tf);
        camera_transforms_tf.push_back(camera_tf);
    }

    // DEBUG
    //    visualize_data(rgbv1, rgbv2, rgbv3, depthv1);
    testTransformsROS(argc, argv, camera_transforms_tf);
}



void testTransformsROS(int argc, char** argv, const std::vector<tf::Transform>& transforms){
    ros::init(argc, argv, "xs_data_reader");
    ros::NodeHandle n("~");

    tf::TransformBroadcaster br1;
    tf::TransformBroadcaster br2;
    tf::Transform transform;

    ros::Rate rate(50.0f);

    while (n.ok()) {
        br1.sendTransform(tf::StampedTransform(transforms[0].inverse(), ros::Time::now(), "xtion", "XS1"));
        br2.sendTransform(tf::StampedTransform(transforms[1].inverse(), ros::Time::now(), "xtion", "XS2"));
        rate.sleep();
    }
}

// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include <cstdlib>
#include <iostream>

#include "CLIEngine.h"
#include "../libs/ITMLib/ITMLibDefines.h"
#include "../libs/ITMLib/Core/ITMBasicEngine.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <Eigen/Eigen>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "InfiniTAMCheck.h"

using namespace InfiniTAM::Engine;
using namespace ITMLib;
Eigen::Vector3d t;
Eigen::Quaterniond q;
//ros::Time tt;
ITMMainEngine *mainEngine;
int count = 0;

bool save() {
    time_t seconds = time(0);
    std::stringstream ss;
    ss << seconds << ".stl";
    std::cout << ss.str() << std::endl;

    mainEngine->SaveSceneToMesh(ss.str().c_str());
}

cv_bridge::CvImagePtr cv_rgb_image_;
int cnt = 0;

void imageCallback(const sensor_msgs::Image::ConstPtr &msg) {
    if (cnt++ < 25) return;
    if (cnt == 4000) std::thread(save).join();
    std::cout << "image\t" << msg->header.stamp << "\t" << msg->encoding << "\t"
              << (int) msg->is_bigendian << std::endl;

    cv_bridge::CvImagePtr cv_depth_image_ = cv_bridge::toCvCopy(msg, msg->encoding);
    // When streaming raw images from Gazebo.
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        constexpr double kDepthScalingFactor = 1000.0;
        // When doing live streaming from the camera.
        (cv_depth_image_->image)
                .convertTo(cv_depth_image_->image, CV_16UC1, kDepthScalingFactor);
    }
//    std::vector<int> compression_params; // Stores the compression parameters
//    compression_params.push_back(CV_IMWRITE_PXM_BINARY); // Set to PXM compression
//    compression_params.push_back(0); // Set type of PXM in our case PGM
//    char buff[10], buff2[10], buff3[10];
//    snprintf(buff, sizeof(buff), "%04i.pgm", count);
//    snprintf(buff2, sizeof(buff2), "%04i.txt", count);
//    snprintf(buff3, sizeof(buff3), "%04i.ppm", count);
//    std::string imageFilename = buff;
//    std::string poseFilename = buff2;
//    std::string rgbFilename = buff3;
//    cv::imwrite(imageFilename, cv_depth_image_->image, compression_params);
//    cv::imwrite(rgbFilename, cv_rgb_image_->image, compression_params);

    Eigen::Matrix3d R = q.normalized().toRotationMatrix();
    Matrix4f M(R(0, 0), R(1, 0), R(2, 0), 0,
               R(0, 1), R(1, 1), R(2, 1), 0,
               R(0, 2), R(1, 2), R(2, 2), 0,
               t(0), t(1), t(2), 1);
//    ORUtils::Matrix3<float> ORUtils_R(R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2));
//    ORUtils::Matrix3<float> ORUtils_R(R(0, 0), R(1, 0), R(2, 0), R(0, 1), R(1, 1), R(2, 1), R(0, 2), R(1, 2), R(2, 2));
//    ORUtils::Vector3<float> ORUtils_t(t(0), t(1), t(2));
//    std::ofstream log(buff2, std::ios_base::out);
//    log <<  << ' ' << R(0, 1) << ' ' << R(0, 2) << ' ' << t(0) << ' ' <<
//        R(1, 0) << ' ' << R(1, 1) << ' ' << R(1, 2) << ' ' << t(1) << ' ' <<
//        R(2, 0) << ' ' << R(2, 1) << ' ' << R(2, 2) << ' ' << t(2) << ' ' <<
//        0 << ' ' << 0 << ' ' << 0 << ' ' << 1 << '\n';
    CLIEngine::Instance()->Run(cv_depth_image_, M);

    printf("%f\t%f\t%f\t%f\n", t(0), t(1), t(2), InfiniTAMCheck(mainEngine, -3.3, -2.5, -1.4, 0.1));





    count++;
}

void poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    std::cout << "poseStamp\t" << msg->header.stamp << std::endl;
    t = Eigen::Vector3d(msg->pose.position.x,
                        msg->pose.position.y,
                        msg->pose.position.z);
    q = Eigen::Quaterniond(msg->pose.orientation.w,
                           msg->pose.orientation.x,
                           msg->pose.orientation.y,
                           msg->pose.orientation.z);
//    std::cout << (ros::Time::now() - tt).toSec() << std::endl;
//    if ((ros::Time::now() - tt).toSec() > 30) {
//        tt = ros::Time::now();
//        std::thread(save).join();
//    }

}

void rgbcallback(const sensor_msgs::Image::ConstPtr &msg) {
    std::cout << "poseStamp\t" << msg->header.stamp << std::endl;
    cv_rgb_image_ = cv_bridge::toCvCopy(msg, msg->encoding);
    // When streaming raw images from Gazebo.
//    if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
//        constexpr double kDepthScalingFactor = 1000.0;
//         When doing live streaming from the camera.
    cv::cvtColor(cv_rgb_image_->image, cv_rgb_image_->image, CV_GRAY2RGB);
//        (cv_rgb_image_->image)
//                .convertTo(cv_rgb_image_->image, CV_8UC3);
//    }
}

void readCalibFile(const char *calibFilename, ITMRGBDCalib &calib) {
    if (!calibFilename || strlen(calibFilename) == 0) {
        printf("Calibration filename not specified. Using default parameters.\n");
        return;
    }

    if (!readRGBDCalib(calibFilename, calib))
        DIEWITHEXCEPTION("error: path to the calibration file was specified but data could not be read");
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "infinitam");
    ros::NodeHandle node;
//    tt = ros::Time::now();
    ros::Subscriber subDepthImage = node.subscribe("/open_quadtree_mapping/depth", 1, imageCallback);
//    ros::Subscriber subRGBImage = node.subscribe("/mv_25001498/image_raw", 1, rgbcallback);
    ros::Subscriber subPose = node.subscribe("/vins_estimator/camera_pose", 1, poseStampedCallback);
    const char *calibFile = "../calib.txt";

//    ros::Subscriber subDepthImage = node.subscribe("/tum_warpper/float_gt_depth", 1, imageCallback);
//    ros::Subscriber subRGBImage = node.subscribe("/tum_warpper/image", 1, rgbcallback);
//    ros::Subscriber subPose = node.subscribe("/tum_warpper/cur_pose", 1, poseStampedCallback);
//    const char *calibFile = "/home/tommy/calib_new.txt";



    ITMLib::ITMRGBDCalib calib;
    readCalibFile(calibFile, calib);
    float mu = 0.5f;
    float voxelSize = 0.1f;
    float viewFrustum_min= 0.01f;
    float viewFrustum_max=5.0f;
    ITMLibSettings *internalSettings = new ITMLibSettings(mu, 100, voxelSize, viewFrustum_min, viewFrustum_max);

    mainEngine = new ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(
            internalSettings, calib, calib.intrinsics_rgb.imgSize, calib.intrinsics_d.imgSize
    );

    CLIEngine::Instance()->Initialise(calib, mainEngine, internalSettings->deviceType);

    ros::spin();

    CLIEngine::Instance()->Shutdown();

    delete mainEngine;
    delete internalSettings;
    return 0;
}

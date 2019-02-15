// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include <cstdlib>
#include <iostream>
#include <string>

#include "CLIEngine.h"
#include "InfiniTAMCheck.h"
#include "../libs/ITMLib/ITMLibDefines.h"
#include "../libs/ITMLib/Core/ITMBasicEngine.h"
#include <thread>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tuple>
#include <queue>
using namespace InfiniTAM::Engine;
using namespace ITMLib;

ros::Subscriber _subDepthImage, _subPose;
ros::Publisher  _pub_mesh_vis,  _pub_voxel_vis;

// Eigen::Vector3d    _position;
// Eigen::Matrix3d    _rotation;
// Eigen::Quaterniond _orientation;

ITMMainEngine * _mainEngine;

std::queue<std::tuple<ros::Time, Eigen::Vector3d, Eigen::Quaterniond>> transformQueue;

void visualizeMesh (const std::vector<Vector3f>  * meshes);
void visualizeVoxel(const std::vector<Vector3f>  * voxeles);

template<class TVoxel=ITMVoxel>
std::vector<Vector3f> * getAllVoxelsInThreshold(ITMLib::ITMMainEngine *mainEngine, float threshold = 0.5) {
    ITMLib::ITMScene<TVoxel, ITMVoxelIndex> *scene
            = dynamic_cast<ITMLib::ITMBasicEngine<ITMVoxel_s, ITMVoxelIndex> *>(mainEngine)->getScene();
    TVoxel *localVBA = scene->localVBA.GetVoxelBlocks_CPU();
    const ITMHashEntry *hashTable = scene->index.GetEntries_CPU();
    std::vector<Vector3f> * vp = new std::vector<Vector3f>();
    int noTotalEntries = scene->index.noTotalEntries;

    for (int entryId = 0; entryId < noTotalEntries; entryId++) {
        Vector3i globalPos;
        const ITMHashEntry &currentHashEntry = hashTable[entryId];

        if (currentHashEntry.ptr < 0) continue;

        globalPos = currentHashEntry.pos.toInt() * SDF_BLOCK_SIZE;
        TVoxel *localVoxelBlock = &(localVBA[currentHashEntry.ptr * (SDF_BLOCK_SIZE3)]);
        for (int z = 0; z < SDF_BLOCK_SIZE; z++)
            for (int y = 0; y < SDF_BLOCK_SIZE; y++)
                for (int x = 0; x < SDF_BLOCK_SIZE; x++) {
                    float sdf = TVoxel::valueToFloat(localVoxelBlock[x + y * SDF_BLOCK_SIZE +
                                                                     z * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE].sdf);
                    if (sdf > -threshold && sdf < threshold) {
                        Vector3f tmp = ((globalPos + Vector3i(x, y, z)).toFloat()
                                        + Vector3f(0.5, 0.5, 0.5)) * scene->sceneParams->voxelSize;
                        vp->push_back(tmp);
                    }
                }
    }
    return vp;
}

std::vector<Vector3f> *getTriangleMeshPoints(ITMLib::ITMMainEngine *mainEngine) {
    return dynamic_cast<ITMLib::ITMBasicEngine<ITMVoxel_s, ITMVoxelIndex> *>(mainEngine)->getTriangleMeshPoints();
}

void save(ITMMainEngine * mainEngine) {
    time_t seconds = time(0);
    std::stringstream ss;
    ss << seconds << ".stl";
    std::cout << ss.str() << std::endl;

    mainEngine->SaveSceneToMesh(ss.str().c_str());
}

void visualize_mesh(ITMMainEngine * mainEngine) {
    //std::cout<< "1" <<std::endl;
    auto meshes = getTriangleMeshPoints(mainEngine);
    visualizeMesh(meshes);
    //std::cout<< meshes->size() <<std::endl;

    delete meshes;
}

void visualize_voxel(ITMMainEngine * mainEngine, double thr) {
    //std::cout<< "1" <<std::endl;
    auto voxels = getAllVoxelsInThreshold(mainEngine, thr);
    visualizeVoxel(voxels);
    //std::cout<< meshes->size() <<std::endl;

    delete voxels;
}

std::string _pkg_path;
double _mu, _voxelSize, _viewFrustum_min, _viewFrustum_max;
int _depth_img_cnt = 0;

cv_bridge::CvImagePtr _cv_rgb_image;
cv_bridge::CvImagePtr _cv_depth_image;

void visualizeMesh(const std::vector<Vector3f> * meshes)
{   
    visualization_msgs::Marker mesh_ros;
    mesh_ros.header.stamp       = ros::Time::now();
    mesh_ros.header.frame_id    = "map";

    mesh_ros.ns = "infinitam_ros/mesh";
    mesh_ros.id = 0;
    mesh_ros.type = visualization_msgs::Marker::TRIANGLE_LIST;

    mesh_ros.pose.orientation.x = 0.0;
    mesh_ros.pose.orientation.y = 0.0;
    mesh_ros.pose.orientation.z = 0.0;
    mesh_ros.pose.orientation.w = 1.0;
    mesh_ros.color.a = 1.0;
    mesh_ros.color.r = 1.0;
    mesh_ros.color.g = 1.0;
    mesh_ros.color.b = 1.0;

    mesh_ros.scale.x = 1.0;
    mesh_ros.scale.y = 1.0;
    mesh_ros.scale.z = 1.0;

    int mesh_size = meshes->size();// / 3;
    geometry_msgs::Point pt;
    for(int i = 0; i < mesh_size; i++ )
    {   
        pt.x = (*meshes)[i].x;
        pt.y = (*meshes)[i].y;
        pt.z = (*meshes)[i].z;
        mesh_ros.points.push_back(pt);
    }

    _pub_mesh_vis.publish(mesh_ros);
}

void visualizeVoxel(const std::vector<Vector3f> * voxeles)
{   
    visualization_msgs::Marker mesh_ros;
    mesh_ros.header.stamp       = ros::Time::now();
    mesh_ros.header.frame_id    = "map";

    mesh_ros.ns = "infinitam_ros/voxel";
    mesh_ros.id = 0;
    mesh_ros.type = visualization_msgs::Marker::CUBE_LIST;

    mesh_ros.pose.orientation.x = 0.0;
    mesh_ros.pose.orientation.y = 0.0;
    mesh_ros.pose.orientation.z = 0.0;
    mesh_ros.pose.orientation.w = 1.0;
    mesh_ros.color.a = 1.0;
    mesh_ros.color.r = 1.0;
    mesh_ros.color.g = 0.0;
    mesh_ros.color.b = 0.0;

    mesh_ros.scale.x = _voxelSize;
    mesh_ros.scale.y = _voxelSize;
    mesh_ros.scale.z = _voxelSize;

    int mesh_size = voxeles->size();
    geometry_msgs::Point pt;
    for(int i = 0; i < mesh_size; i++ )
    {   
        pt.x = (*voxeles)[i].x;
        pt.y = (*voxeles)[i].y;
        pt.z = (*voxeles)[i].z;
        mesh_ros.points.push_back(pt);
    }

    _pub_mesh_vis.publish(mesh_ros);
}

void imageCallback(const sensor_msgs::Image::ConstPtr &msg) {
    if (_depth_img_cnt ++ < 25) return;
    if (_depth_img_cnt % 30 == 0) 
    {   
        //printf("%d\t%d\n", _depth_img_cnt, meshes.size());
        std::thread(visualize_voxel, _mainEngine, 0.5).detach();//join();//;
    }
    if (_depth_img_cnt % 100 == 0) 
    {
        std::cout  << "Creating Thread" << std::endl;
        std::thread(visualize_mesh, _mainEngine).detach();//join();//;
        //printf("%d\t%d\n", _depth_img_cnt, meshes.size());
    }

    //if (cnt == 1500) std::thread(save).join();
    std::cout << "Image timestamp\t" << msg->header.stamp << "\t" << std::endl;

    bool newPos = false;
    Eigen::Vector3d _position;
    Eigen::Quaterniond _orientation;
    ros::Time _poseTime;
    while (!transformQueue.empty() && std::get<0>(transformQueue.front()) <= msg->header.stamp) {
        _poseTime = std::get<0>(transformQueue.front());
        _position = std::get<1>(transformQueue.front());
        _orientation = std::get<2>(transformQueue.front());
        transformQueue.pop();
        newPos = true;
    }
    if (!newPos) return;
    std::cout << "Corresponding pose timestamp\t" << _poseTime <<"\t" << transformQueue.size() << "\t" << std::endl;


    _cv_depth_image = cv_bridge::toCvCopy(msg, msg->encoding);
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) 
    {
        constexpr double kDepthScalingFactor = 1000.0;
        (_cv_depth_image->image).convertTo(_cv_depth_image->image, CV_16UC1, kDepthScalingFactor);
    }

//    std::vector<int> compression_params; // Stores the compression parameters
//    compression_params.push_back(CV_IMWRITE_PXM_BINARY); // Set to PXM compression
//    compression_params.push_back(0); // Set type of PXM in our case PGM
//    char buff[10], buff2[10], buff3[10];
//    std::string imageFilename = buff;
//    std::string poseFilename = buff2;
//    std::string rgbFilename = buff3;
//    cv::imwrite(imageFilename, _cv_depth_image->image, compression_params);
//    cv::imwrite(rgbFilename, _cv_rgb_image->image, compression_params);
    Eigen::Matrix3d _rotation    = _orientation.normalized().toRotationMatrix();

    Matrix4f camera_pose( _rotation(0, 0), _rotation(1, 0), _rotation(2, 0), 0,
                          _rotation(0, 1), _rotation(1, 1), _rotation(2, 1), 0,
                          _rotation(0, 2), _rotation(1, 2), _rotation(2, 2), 0,
                          _position(0)   , _position(1)   , _position(2),    1 
                        );

    CLIEngine::Instance()->Run(_cv_depth_image, camera_pose);
    //InfiniTAMCheck(_mainEngine, -3.3, -2.5, -1.4, 0.1);
    //printf("%f\t%f\t%f\t%f\n", t(0), t(1), t(2), InfiniTAMCheck(_mainEngine, -3.3, -2.5, -1.4, 0.1) );
}

//void poseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
void poseStampedCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    //std::cout << "poseStamp\t" << msg->header.stamp << std::endl;
    Eigen::Vector3d _position    = Eigen::Vector3d(msg->pose.pose.position.x,
                                   msg->pose.pose.position.y,
                                   msg->pose.pose.position.z);

    Eigen::Quaterniond _orientation = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                      msg->pose.pose.orientation.x,
                                      msg->pose.pose.orientation.y,
                                      msg->pose.pose.orientation.z);
    

    transformQueue.push(std::make_tuple(msg->header.stamp, _position, _orientation));
}

void rgbcallback(const sensor_msgs::Image::ConstPtr &msg) {
    //std::cout << "poseStamp\t" << msg->header.stamp << std::endl;
    _cv_rgb_image = cv_bridge::toCvCopy(msg, msg->encoding);
    cv::cvtColor(_cv_rgb_image->image, _cv_rgb_image->image, CV_GRAY2RGB);
}

void readCalibFile(const char *calibFilename, ITMRGBDCalib &calib) {
    if (!calibFilename || strlen(calibFilename) == 0) {
        printf("Calibration filename not specified. Using default parameters.\n");
        return;
    }

    if (!readRGBDCalib(calibFilename, calib))
        DIEWITHEXCEPTION("error: path to the calibration file was specified but data could not be read");
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "infinitam_ros_node_demo");
    ros::NodeHandle nh("~");

    nh.param( "ros_node/pkg_path", _pkg_path, std::string("") );
    nh.param( "fusion_param/mu",              _mu,        0.5 );
    nh.param( "fusion_param/voxelSize",       _voxelSize, 0.1 );
    nh.param( "fusion_param/viewFrustum_min", _viewFrustum_min, 0.01 );
    nh.param( "fusion_param/viewFrustum_max", _viewFrustum_max, 5.0  );

    /*_subDepthImage = nh.subscribe("/open_quadtree_mapping/depth", 1, imageCallback);
    _subPose       = nh.subscribe("/vins_estimator/camera_pose" ,   1, poseStampedCallback);*/

    /*_subDepthImage = nh.subscribe("/icl_nuim_wapper/gt_depth", 1, imageCallback);
    _subPose       = nh.subscribe("/icl_nuim_wapper/cur_pose" , 1, poseStampedCallback);*/
    
    _subDepthImage = nh.subscribe("/sgbm_ros_node/depth_image",   10, imageCallback);
    _subPose       = nh.subscribe("/vins_estimator/camera_pose",  10, poseStampedCallback);

    _pub_mesh_vis  = nh.advertise<visualization_msgs::Marker>("infinitam_mesh_vis", 1);

    _pkg_path += std::string("/calib.txt");

    const char * calibFile = _pkg_path.c_str();
    ITMLib::ITMRGBDCalib calib;
    readCalibFile(calibFile, calib);
    
    ITMLibSettings * _internalSettings = new ITMLibSettings(_mu, 10, _voxelSize, _viewFrustum_min, _viewFrustum_max);

    _mainEngine = new ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(
            _internalSettings, calib, calib.intrinsics_rgb.imgSize, calib.intrinsics_d.imgSize );

    CLIEngine::Instance()->Initialise(calib, _mainEngine, _internalSettings->deviceType);

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
    //ros::spin();

    std::thread(save, _mainEngine).join();
    CLIEngine::Instance()->Shutdown();

    delete _mainEngine;
    delete _internalSettings;
    return 0;
}

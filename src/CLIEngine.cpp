// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#include "CLIEngine.h"

#include <string.h>

#include "../libs/ORUtils/FileUtils.h"
#include "../libs/ITMLib/Core/ITMBasicEngine.h"
#include "../libs/ITMLib/ITMLibDefines.h"
#include "../libs/ITMLib/Objects/Scene/ITMRepresentationAccess.h"

using namespace InfiniTAM::Engine;
using namespace ITMLib;

CLIEngine *CLIEngine::instance;

void CLIEngine::Initialise(ITMLib::ITMRGBDCalib calib, ITMMainEngine *mainEngine,
                           ITMLibSettings::DeviceType deviceType) {
    this->calib = calib;
    this->mainEngine = mainEngine;

    this->currentFrameNo = 0;

    bool allocateGPU = false;
    if (deviceType == ITMLibSettings::DEVICE_CUDA) allocateGPU = true;

    inputRGBImage = new ITMUChar4Image(calib.intrinsics_rgb.imgSize, true, allocateGPU);
    inputRawDepthImage = new ITMShortImage(calib.intrinsics_d.imgSize, true, allocateGPU);
//    inputIMUMeasurement = new ITMIMUMeasurement();

#ifndef COMPILE_WITHOUT_CUDA
    ORcudaSafeCall(cudaThreadSynchronize());
#endif

    sdkCreateTimer(&timer_instant);
    sdkCreateTimer(&timer_average);

    sdkResetTimer(&timer_average);

    printf("initialised.\n");
}

bool CLIEngine::ProcessFrame() {
//    if (!imageSource->hasMoreImages()) return false;
//    imageSource->getImages(inputRGBImage, inputRawDepthImage);


    sdkResetTimer(&timer_instant);
    sdkStartTimer(&timer_instant);
    sdkStartTimer(&timer_average);

    //actual processing on the mailEngine


    mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);
#ifndef COMPILE_WITHOUT_CUDA
    ORcudaSafeCall(cudaThreadSynchronize());
#endif
    sdkStopTimer(&timer_instant);
    sdkStopTimer(&timer_average);

    float processedTime_inst = sdkGetTimerValue(&timer_instant);
    float processedTime_avg = sdkGetAverageTimerValue(&timer_average);

    printf("frame %i: time %.2f, avg %.2f\n", currentFrameNo, processedTime_inst, processedTime_avg);

    currentFrameNo++;

    return true;
}

void CLIEngine::Run(cv_bridge::CvImagePtr cv_depth_image_, Matrix4f M) {

    // TODO: NO TEMPLATE
    ITMTrackingState *trackingState = dynamic_cast<ITMBasicEngine<ITMVoxel, ITMVoxelIndex> *>(mainEngine)->getTrackingState();
    trackingState->pose_d->SetInvM(M);
    trackingState->trackerResult = ITMTrackingState::TRACKING_GOOD;

    cv::Size depth_size = cv_depth_image_->image.size();
    short *raw_depth_infinitam = inputRawDepthImage->GetData(MEMORYDEVICE_CPU);

    uint depth_rows = depth_size.height;
    uint depth_cols = depth_size.width;
    for (size_t i = 0; i < depth_rows * depth_cols; ++i) {
        raw_depth_infinitam[i] =
                (((int) (cv_depth_image_->image.data[2 * i + 1]) << 8) & 0xFF00) |
                (cv_depth_image_->image.data[2 * i] & 0xFF);
    }
//    rgb->SetFrom(cached_rgb, ORUtils::MemoryBlock<Vector4u>::CPU_TO_GPU);
    ProcessFrame();
}

//template<class TVoxel>
//TVoxel CLIEngine::check(float x, float y, float z) {
//
//}

void CLIEngine::Shutdown() {
    sdkDeleteTimer(&timer_instant);
    sdkDeleteTimer(&timer_average);

    delete inputRGBImage;
    delete inputRawDepthImage;
//    delete inputIMUMeasurement;

    delete instance;
}

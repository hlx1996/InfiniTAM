// Copyright 2014-2017 Oxford University Innovation Limited and the authors of InfiniTAM

#pragma once

#include <cv_bridge/cv_bridge.h>
#include "../libs/ITMLib/Core/ITMMainEngine.h"
#include "../libs/ITMLib/Utils/ITMLibSettings.h"
#include "../libs/ORUtils/FileUtils.h"
#include "../libs/ORUtils/NVTimer.h"
#include "../libs/ITMLib/ITMLibDefines.h"
#include "../libs/ITMLib/Objects/Scene/ITMScene.h"
#include "../libs/ITMLib/Objects/Scene/ITMRepresentationAccess.h"
#include "../libs/ITMLib/Core/ITMBasicEngine.h"

namespace InfiniTAM {
    namespace Engine {
        class CLIEngine {
            static CLIEngine *instance;

            ITMLib::ITMRGBDCalib calib;
            ITMLib::ITMLibSettings internalSettings;
            ITMLib::ITMMainEngine *mainEngine;

            StopWatchInterface *timer_instant;
            StopWatchInterface *timer_average;

        private:
            ITMUChar4Image *inputRGBImage;
            ITMShortImage *inputRawDepthImage;
//            ITMLib::ITMIMUMeasurement *inputIMUMeasurement;

            int currentFrameNo;
        public:
            static CLIEngine *Instance(void) {
                if (instance == NULL) instance = new CLIEngine();
                return instance;
            }

            float processedTime;

            void Initialise(ITMLib::ITMRGBDCalib calib,
                            ITMLib::ITMMainEngine *mainEngine,
                            ITMLib::ITMLibSettings::DeviceType deviceType);

            void Shutdown();

            void Run(cv_bridge::CvImagePtr cv_depth_image_, Matrix4f M);

            bool ProcessFrame();
        };
    }
}

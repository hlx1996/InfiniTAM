//
// Created by tommy on 1/31/19.
//

#include "../include/InfiniTAMCheck.h"
#include "../include/CLIEngine.h"
#include "../libs/ITMLib/ITMLibDefines.h"
#include "../libs/ITMLib/Objects/Scene/ITMScene.h"
#include "../libs/ITMLib/Core/ITMBasicEngine.h"
#include "../libs/ITMLib/Objects/Scene/ITMRepresentationAccess.h"

template<class TVoxel=ITMVoxel>
__device__ void InfiniTAMCheck_device(ITMLib::ITMScene<TVoxel, ITMVoxelIndex> *scene,
                                      ITMHashEntry *hashTable, TVoxel *localVBA,
                                      float x, float y, float z,
                                      float voxelSize, float requireSize, float *answer) {

    Vector3f point(x, y, z);
    int times = int(requireSize / voxelSize);

    int require_block_size = SDF_BLOCK_SIZE / times;
    float oneOverVoxelSizeBlockSize = 1.0f / (voxelSize * SDF_BLOCK_SIZE);
    float oneOverRequireSize = 1.0f / requireSize;

    Vector3s blockPos = TO_SHORT_FLOOR3(point * oneOverVoxelSizeBlockSize);
    Vector3s posInBlock =
            (TO_SHORT_FLOOR3(point * oneOverRequireSize) - blockPos * require_block_size) * times;
    int hashIdx = hashIndex(blockPos);

    ITMHashEntry hashEntry = hashTable[hashIdx];
    bool isFound = false;
    if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1) {
        isFound = true;
        TVoxel *localVoxelBlock = &(localVBA[hashEntry.ptr * (SDF_BLOCK_SIZE3)]);
        float ans = 1e100;
        for (int i = 0; i < times; i++)
            for (int j = 0; j < times; j++)
                for (int k = 0; k < times; k++) {
                    float tmp = TVoxel::valueToFloat(
                            localVoxelBlock[(posInBlock[0] + i) + (posInBlock[1] + j) * SDF_BLOCK_SIZE +
                                            (posInBlock[2] + k) * SDF_BLOCK_SIZE * SDF_BLOCK_SIZE].sdf);
                    if (fabs(tmp) < fabs(ans)) ans = tmp;
                }

        *answer = ans;
        return;
    }

    if (!isFound) {
        if (hashEntry.ptr >= -1) //search excess list only if there is no room in ordered part
        {
            while (hashEntry.offset >= 1) {
                hashIdx = SDF_BUCKET_NUM + hashEntry.offset - 1;
                hashEntry = hashTable[hashIdx];

                if (IS_EQUAL3(hashEntry.pos, blockPos) && hashEntry.ptr >= -1) {
                    TVoxel *localVoxelBlock = &(localVBA[hashEntry.ptr * (SDF_BLOCK_SIZE3)]);
                    float ans = 1e100;
                    for (int i = 0; i < times; i++)
                        for (int j = 0; j < times; j++)
                            for (int k = 0; k < times; k++) {
                                float tmp = TVoxel::valueToFloat(localVoxelBlock[(posInBlock[0] + i)
                                                                                 + (posInBlock[1] + j) * SDF_BLOCK_SIZE
                                                                                 +
                                                                                 (posInBlock[2] + k) * SDF_BLOCK_SIZE *
                                                                                 SDF_BLOCK_SIZE].sdf);
                                if (fabs(tmp) < fabs(ans)) ans = tmp;
                            }
                    *answer = ans;
                    return;
                }
            }
        }
    }
    *answer = 1e100;
//    return TVoxel();
}

template<class TVoxel=ITMVoxel>
__global__ void InfiniTAMCheck_global(ITMLib::ITMScene<TVoxel, ITMVoxelIndex> *scene,
                                      ITMHashEntry *hashTable, TVoxel *localVBA,
                                      float x, float y, float z,
                                      float voxelSize, float requireSize, float *answer) {
    InfiniTAMCheck_device(scene, hashTable, localVBA, x, y, z, voxelSize, requireSize, answer);

}

float InfiniTAMCheck(ITMLib::ITMMainEngine *mainEngine, float x, float y, float z, float requireSize) {
    ITMLib::ITMScene<ITMVoxel_s, ITMVoxelIndex> *scene
            = dynamic_cast<ITMLib::ITMBasicEngine<ITMVoxel_s, ITMVoxelIndex> *>(mainEngine)->getScene();
    ITMHashEntry *hashTable = scene->index.GetEntries();

    ITMVoxel_s *localVBA = scene->localVBA.GetVoxelBlocks();
    float *answer_device, answer_host;
    cudaMalloc(&answer_device, sizeof(float));
    InfiniTAMCheck_global << < 1, 1 >> >
                                  (scene, hashTable, localVBA, x, y, z,
                                          scene->sceneParams->voxelSize, requireSize, answer_device);
    cudaMemcpy(&answer_host, answer_device, sizeof(float), cudaMemcpyDeviceToHost);
    ORcudaSafeCall(cudaThreadSynchronize());
    if (answer_host > 1 || answer_host < -1) return 1e100;
    if (answer_host > 0.9999 || answer_host < -0.9999) return 100;
    return answer_host * scene->sceneParams->mu;
}

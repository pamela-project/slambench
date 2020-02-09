/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef FRAMEWORK_SHARED_INCLUDE_METRICS_MEMORY_UTILS_CUDAMONITOR_H_
#define FRAMEWORK_SHARED_INCLUDE_METRICS_MEMORY_UTILS_CUDAMONITOR_H_

#include <cstdlib>
#include <iostream>

typedef int cudaError_t ;
typedef cudaError_t (*cudaMemGetInfo_t)( 	size_t *  	,  	size_t *  	);
typedef struct CUuuid_st {
     char bytes[16];
 } CUuuid;

typedef struct CUuuid_st cudaUUID_t;
typedef struct {
    char name[256];
    cudaUUID_t uuid;
    size_t totalGlobalMem;
    size_t sharedMemPerBlock;
    int regsPerBlock;
    int warpSize;
    size_t memPitch;
    int maxThreadsPerBlock;
    int maxThreadsDim[3];
    int maxGridSize[3];
    int clockRate;
    size_t totalConstMem;
    int major;
    int minor;
    size_t textureAlignment;
    size_t texturePitchAlignment;
    int deviceOverlap;
    int multiProcessorCount;
    int kernelExecTimeoutEnabled;
    int integrated;
    int canMapHostMemory;
    int computeMode;
    int maxTexture1D;
    int maxTexture1DMipmap;
    int maxTexture1DLinear;
    int maxTexture2D[2];
    int maxTexture2DMipmap[2];
    int maxTexture2DLinear[3];
    int maxTexture2DGather[2];
    int maxTexture3D[3];
    int maxTexture3DAlt[3];
    int maxTextureCubemap;
    int maxTexture1DLayered[2];
    int maxTexture2DLayered[3];
    int maxTextureCubemapLayered[2];
    int maxSurface1D;
    int maxSurface2D[2];
    int maxSurface3D[3];
    int maxSurface1DLayered[2];
    int maxSurface2DLayered[3];
    int maxSurfaceCubemap;
    int maxSurfaceCubemapLayered[2];
    size_t surfaceAlignment;
    int concurrentKernels;
    int ECCEnabled;
    int pciBusID;
    int pciDeviceID;
    int pciDomainID;
    int tccDriver;
    int asyncEngineCount;
    int unifiedAddressing;
    int memoryClockRate;
    int memoryBusWidth;
    int l2CacheSize;
    int maxThreadsPerMultiProcessor;
    int streamPrioritiesSupported;
    int globalL1CacheSupported;
    int localL1CacheSupported;
    size_t sharedMemPerMultiprocessor;
    int regsPerMultiprocessor;
    int managedMemory;
    int isMultiGpuBoard;
    int multiGpuBoardGroupID;
    int singleToDoublePrecisionPerfRatio;
    int pageableMemoryAccess;
    int concurrentManagedAccess;
    int computePreemptionSupported;
    int canUseHostPointerForRegisteredMem;
    int cooperativeLaunch;
    int cooperativeMultiDeviceLaunch;
    int pageableMemoryAccessUsesHostPageTables;
    int directManagedMemAccessFromHost;
} cudaDeviceProp;
typedef cudaError_t (*cudaGetDeviceProperties_t)( 	cudaDeviceProp *  	,  	int 	);

class CUDAMonitor {
public :
	CUDAMonitor () : libcuda_cudaMemGetInfo(nullptr),
                     libcuda_cudaGetDeviceProperties(nullptr)
	{
		Init();
	}
private :
	cudaMemGetInfo_t libcuda_cudaMemGetInfo  ;
    cudaGetDeviceProperties_t libcuda_cudaGetDeviceProperties;
	bool Init();
public :
	size_t getUsedBytes() ;
	bool IsActive() ;
    std::string device_name = "";



};



#endif /* FRAMEWORK_SHARED_INCLUDE_METRICS_MEMORY_UTILS_CUDAMONITOR_H_ */

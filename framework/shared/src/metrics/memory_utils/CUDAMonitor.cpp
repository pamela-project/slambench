/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "metrics/memory_utils/CUDAMonitor.h"
#include <iostream>
#include <dlfcn.h>

bool CUDAMonitor::IsActive() {
	 return libcuda_cudaMemGetInfo != nullptr;
}

bool CUDAMonitor::Init() {
	 libcuda_cudaMemGetInfo = (cudaMemGetInfo_t)dlsym(RTLD_NEXT, "cudaMemGetInfo");
		if (libcuda_cudaMemGetInfo == nullptr) {
			std::cerr << "*** CUDA Monitor: cudaMemGetInfo is not available" << std::endl;
		}
    libcuda_cudaGetDeviceProperties = (cudaGetDeviceProperties_t)dlsym(RTLD_NEXT, "cudaGetDeviceProperties");
    if (libcuda_cudaGetDeviceProperties == nullptr) {
        std::cerr << "*** CUDA Monitor: cudaGetDeviceProperties is not available" << std::endl;
    } else {
        cudaDeviceProp prop;
        libcuda_cudaGetDeviceProperties(&prop, 0);
        device_name = prop.name;
    }
	 return libcuda_cudaMemGetInfo != nullptr;
}



size_t CUDAMonitor::getUsedBytes() {

	if (libcuda_cudaMemGetInfo == nullptr) {
		return 0;
	}
	// show memory usage of GPU

	size_t free_byte ;

	size_t total_byte ;

	cudaError_t error = libcuda_cudaMemGetInfo(&free_byte, &total_byte);


	if (error != 0) {
		return 0.0;
	}

	double free_db = (double)free_byte ;

	double total_db = (double)total_byte ;

	double used_db = total_db - free_db ;

	return used_db ;

}



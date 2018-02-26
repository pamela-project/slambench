/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef FRAMEWORK_SHARED_INCLUDE_METRICS_MEMORY_UTILS_CUDAMONITOR_H_
#define FRAMEWORK_SHARED_INCLUDE_METRICS_MEMORY_UTILS_CUDAMONITOR_H_

#include <cstdlib>

typedef int cudaError_t ;
typedef cudaError_t (*cudaMemGetInfo_t)( 	size_t *  	,  	size_t *  	);


class CUDAMonitor {
public :
	CUDAMonitor () : libcuda_cudaMemGetInfo(nullptr) {
		Init();
	}
private :
	cudaMemGetInfo_t libcuda_cudaMemGetInfo  ;
	bool Init();
public :
	size_t getUsedBytes() ;
	bool IsActive() ;



};



#endif /* FRAMEWORK_SHARED_INCLUDE_METRICS_MEMORY_UTILS_CUDAMONITOR_H_ */

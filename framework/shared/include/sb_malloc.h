/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef SB_MALLOC_H_
#define SB_MALLOC_H_

#include <map>

#include <cstdio>
#include <stdint.h>
#include <iostream>


#define MEM_ERROR(...) printf( __VA_ARGS__ );
#define MEM_DEBUG(...) printf( __VA_ARGS__ );
#define NO_MEM_DEBUG
#ifdef NO_MEM_DEBUG
#undef  MEM_DEBUG
#define MEM_DEBUG(...)
#endif


#ifdef ANDROID
extern "C" {
void *malloc(size_t size) ;
void free(void* ptr) ;
}
#else
extern "C" {
void *malloc(size_t size) throw();
void free(void* ptr) throw();
}
#endif

namespace slambench {
	namespace memory {
		class MemoryProfiler;
		
		class MemoryData {
			friend class MemoryProfiler;
			public:
				MemoryData() : TotalBytesAllocated(0), MaxBytesAllocated(0), BytesAllocatedAtEndOfFrame(0) {}
			
				ssize_t TotalBytesAllocated;
				ssize_t MaxBytesAllocated;
				ssize_t BytesAllocatedAtEndOfFrame;

				void ResetBytesAllocated () {
					MaxBytesAllocated = BytesAllocatedAtEndOfFrame;
				}
			private:
				void RecordAllocation(ssize_t bytes) 
				{ 
					TotalBytesAllocated += bytes; 
					BytesAllocatedAtEndOfFrame += bytes; 
					if(BytesAllocatedAtEndOfFrame > MaxBytesAllocated) 
					{
						MaxBytesAllocated = BytesAllocatedAtEndOfFrame; 
					}
					MEM_DEBUG("%ld RecordAllocation:%ld\n" , BytesAllocatedAtEndOfFrame , bytes) ;
				}
				
				void RecordFree(ssize_t bytes)
				{
					BytesAllocatedAtEndOfFrame -= bytes;
					MEM_DEBUG("%ld RecordFree:%ld\n" , BytesAllocatedAtEndOfFrame , bytes) ;
				}
		};
		
		class MemoryProfile {
			friend class MemoryProfiler;
			public:
				static MemoryProfile singleton;
				
				void StartFrame(int i);
				void EndFrame();
				
				void StartAlgorithm();
				void EndAlgorithm();
				
				void ResetBytesAllocated () {
					overall_data_.ResetBytesAllocated();
				}
				void ResetGPUBytesAllocated () {
					overall_gpu_data_.ResetBytesAllocated();
				}
				const MemoryData &GetDataForFrame(uint32_t frame_idx) const { return data_.at(frame_idx); }
				const MemoryData &GetDataForCurrentFrame() const { return GetDataForFrame(GetCurrentFrame()); }
				const MemoryData &GetOverallData() const { return overall_data_; }
				
				const MemoryData &GetGPUDataForFrame(uint32_t frame_idx) const { return gpu_data_.at(frame_idx); }
				const MemoryData &GetGPUDataForCurrentFrame() const { return GetGPUDataForFrame(GetCurrentFrame()); }
				const MemoryData &GetOverallGPUData() const { return overall_gpu_data_; }

				bool HasDataForFrame(uint32_t frame_idx) const { return data_.count(frame_idx); }
				
				uint32_t GetCurrentFrame() const { return _frame; };
			private:
				std::map<uint32_t, MemoryData> data_;
				std::map<uint32_t, MemoryData> gpu_data_;
				MemoryData overall_data_;
				MemoryData overall_gpu_data_;
				uint32_t _frame;
				

				MemoryData &DataForFrame(uint32_t frame_idx) { return data_[frame_idx]; }
				MemoryData &GPUDataForFrame(uint32_t frame_idx) { return gpu_data_[frame_idx]; }
				MemoryData &GPUDataForCurrentFrame() { return GPUDataForFrame(GetCurrentFrame()); }
				MemoryData &DataForCurrentFrame() { return DataForFrame(GetCurrentFrame()); }
				MemoryData &OverallData() { return overall_data_; }
				MemoryData &OverallGPUData() { return overall_gpu_data_; }
		};
	}
}

#endif

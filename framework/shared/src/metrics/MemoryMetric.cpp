/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "metrics/MemoryMetric.h"

#include "sb_malloc.h"
#include <cassert>
#include <chrono>
#include <fstream>
#include <string>
#include <unistd.h>
#include <sys/time.h>

using namespace slambench::metrics;

/* CPU Memory Metric */


const slambench::values::ValueDescription &MemoryMetric::GetValueDescription() const {
	return desc_;
}

MemoryMetric::MemoryMetric() : Metric("Memory") , desc_({})
{
	slambench::memory::MemoryProfile::singleton.StartAlgorithm();

//	if (cuda_monitor.IsActive()) {
//		desc_ = slambench::values::ValueDescription({
//						{"CPU_Memory",  slambench::values::VT_U64},
//						{"GPU_Memory",  slambench::values::VT_U64},
//						{"CUDA_Memory", slambench::values::VT_U64}});
//	} else {
		desc_ = slambench::values::ValueDescription({
						{"CPU_Memory",  slambench::values::VT_U64},
						{"GPU_Memory",  slambench::values::VT_U64}});
//	}
}


MemoryMetric::~MemoryMetric() {
	slambench::memory::MemoryProfile::singleton.EndAlgorithm();
}

void MemoryMetric::MeasureStart(Phase* phase)
{

	(void)phase;
	slambench::memory::MemoryProfile::singleton.ResetBytesAllocated();
	slambench::memory::MemoryProfile::singleton.ResetGPUBytesAllocated();



}

void MemoryMetric::MeasureEnd(Phase* phase)
{
	(void)phase;
	assert(!CPU_Usage_.count(phase));
	CPU_Usage_[phase] = slambench::memory::MemoryProfile::singleton.GetOverallData().MaxBytesAllocated ;
	GPU_Usage_[phase] = slambench::memory::MemoryProfile::singleton.GetOverallGPUData().MaxBytesAllocated ;

}

Value *MemoryMetric::GetValue(Phase* phase)
{
	auto cpu_mem = new slambench::values::TypeForVT<slambench::values::VT_U64>::type(CPU_Usage_[phase]);
	auto gpu_mem = new slambench::values::TypeForVT<slambench::values::VT_U64>::type(GPU_Usage_[phase]);

	CPU_Usage_.erase(phase);
	GPU_Usage_.erase(phase);

//	if (cuda_monitor.IsActive()) {
//		auto cuda_mem = new slambench::values::TypeForVT<slambench::values::VT_U64>::type(cuda_monitor.getUsedBytes());
//		return new slambench::values::TypeForVT<slambench::values::VT_COLLECTION>::type({
//			{"CPU_Memory",cpu_mem},
//			{"GPU_Memory",gpu_mem},
//			{"CUDA_Memory",cuda_mem}
//		});
//	} else {
		return new slambench::values::TypeForVT<slambench::values::VT_COLLECTION>::type({
			{"CPU_Memory",cpu_mem},
			{"GPU_Memory",gpu_mem}
		});
//	}

}


const std::string& MemoryMetric::GetDescription() const
{
	static const std::string desc = "Memory in Bytes";
	return desc;
}


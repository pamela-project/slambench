/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "sb_malloc.h"
#include "metrics/DurationMetric.h"

#include <cassert>
#include <chrono>
#include <fstream>
#include <string>
#include <unistd.h>
#include <sys/time.h>

using namespace slambench::metrics;

/* Duration Metric */

DurationMetric::DurationMetric() : Metric("Duration")
{

}

void DurationMetric::MeasureStart(Phase* phase)
{
	assert(!phase_start_.count(phase));
	phase_start_[phase] = getTime();
}

void DurationMetric::MeasureEnd(Phase* phase)
{
	assert(phase_start_.count(phase));
	assert(!phase_end_.count(phase));
	
	phase_end_[phase] = getTime();
}

Value *DurationMetric::GetValue(Phase *phase)
{
	auto value = phase_end_.at(phase) - phase_start_.at(phase);
	phase_end_.erase(phase);
	phase_start_.erase(phase);
	
	// time is recorded in microseconds but report it in full seconds
	return new values::TypedValue<double>(value / 1000000.0);
}


const slambench::values::ValueDescription &DurationMetric::GetValueDescription() const {
	static const slambench::values::ValueDescription desc = slambench::values::VT_DOUBLE;
	return  desc;
}
const std::string &DurationMetric::GetDescription() const
{
	static std::string desc = "Duration of the phase in whole microseconds";
	return desc;
}

uint64_t DurationMetric::getTime() const
{
	auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
	
	// convert time into time in microseconds (1/1000000)
	return std::chrono::duration_cast<std::chrono::duration<uint64_t, std::ratio<1, 1000000>>>(now).count();
}


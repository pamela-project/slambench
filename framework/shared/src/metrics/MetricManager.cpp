/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "metrics/Metric.h"
#include "metrics/MetricManager.h"
#include "metrics/Phase.h"
#include "metrics/DurationMetric.h"

#include <cassert>
#include <sstream>

using namespace slambench::metrics;

MetricManager::MetricManager() : init_phase_(this,"Initialisation"), frame_phase_(this, "Frame"), frame_counter_(0)
{
	strcpy(duration_metric_typeid, typeid(DurationMetric).name());
    frame_phase_duration.clear();
}

MetricManager::~MetricManager() {
	for(auto i : frame_data_) {
		delete i;
	}
}

void MetricManager::AddPhaseMetric(MetricPtr m)
{
	phase_metrics_.push_back(m);
	all_metrics_.insert(m);
}

void MetricManager::AddFrameMetric(MetricPtr m)
{
	frame_metrics_.push_back(m);
	all_metrics_.insert(m);
}

void MetricManager::AddPhase(Phase* p)
{
	phases_.push_back(p);
}

void MetricManager::AddPhase(const std::string &phase_name) {
	AddPhase(new Phase(this, phase_name));
}

void MetricManager::BeginInit()
{
	for(auto i : frame_metrics_) {
		i->MeasureStart(&init_phase_);
	}
}

void MetricManager::EndInit()
{
	for(auto i : frame_metrics_ ) {
		i->MeasureEnd(&init_phase_);
	}

	for(auto i : frame_metrics_) {
		init_data_.insert({&*i, i->GetValue(&init_phase_)});
	}

}

void MetricManager::BeginFrame()
{
	for(auto i : frame_metrics_) {
		i->MeasureStart(&frame_phase_);
	}
}

void MetricManager::BeginPhase(Phase* phase)
{
	for(auto i : phase_metrics_ ) {
		i->MeasureStart(phase);
	}
}

void MetricManager::EndFrame()
{
	for(auto i : frame_metrics_ ) {
		i->MeasureEnd(&frame_phase_);
	}
	
	auto &frame = *GetFrame(frame_counter_);
	auto &frame_data = frame.GetFrameData();
	for(auto i : frame_metrics_) {
		frame_data.insert({&*i, i->GetValue(&frame_phase_)});
        if (!strcmp(duration_metric_typeid,typeid(*i).name())) {
            frame_phase_duration.push_back(dynamic_cast<slambench::values::TypedValue<double>*>(frame_data[&*i])->GetValue());
        }
	}
	// yeaaaahhhhh
	frame.GetPhaseData(&frame_phase_) = frame_data;
	
	for(auto i : phase_metrics_) {
		for(auto j : phases_) {
			frame.GetPhaseData(j).insert({&*i, i->GetValue(j)});
		}
	}
	
	frame_counter_++;
}

FrameData* MetricManager::GetFrame(uint64_t frame_number)
{
	while(frame_data_.size() <= frame_number) {
		frame_data_.push_back(new FrameData());
	}
	assert(frame_data_.at(frame_number) != nullptr);
	return frame_data_.at(frame_number);
}

FrameData* MetricManager::GetLastFrame()
{
	return *frame_data_.rbegin();
}


std::string MetricManager::GetHeader() const
{
	std::ostringstream str;
	
	for(auto m : frame_metrics_) {
		str << "Frame_" << m->GetName() << "\t";
	}
	
	for(auto p : phases_) {
		for(auto m : phase_metrics_) {
			str << p->GetName() << "_" << m->GetName() << "\t";
		}
	}
	return str.str();
}


void MetricManager::EndPhase(Phase* phase)
{
	for(auto i : phase_metrics_ ) {
		i->MeasureEnd(phase);
	}
}

Phase* MetricManager::GetPhase(const std::string& phasename)
{
    for (auto i : phases_) {
        if (i->GetName() == phasename) {
            return i;
        }
    }
    assert(false);
}

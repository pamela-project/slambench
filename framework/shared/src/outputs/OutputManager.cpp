/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "outputs/OutputManager.h"
#include "outputs/OutputManagerWriter.h"

#include "io/sensor/Sensor.h"
#include "io/sensor/GroundTruthSensor.h"
#include "io/sensor/PointCloudSensor.h"
#include "io/format/PointCloud.h"

#include <Eigen/Core>

using namespace slambench::outputs;

OutputManager::~OutputManager()
{
	for(auto &i : output_map_) {
		delete i.second;
	}
}

BaseOutput* OutputManager::GetOutput(const std::string& outputname)
{
	return output_map_.at(outputname);
}

BaseOutput* OutputManager::GetMainOutput(slambench::values::ValueType type)
{
	for(auto &i : output_map_) {
		if(i.second->IsMainOutput() && i.second->GetType() == type) {
			return i.second;
		}
	}
	
	return nullptr;
}

void OutputManager::RegisterOutput(BaseOutput* output)
{
	assert(output != nullptr);
	if(output->IsMainOutput() && GetMainOutput(output->GetType())) {
		delete output_map_[output->GetName()];
		output_map_.erase(output->GetName());
		// throw std::logic_error("A main output for this type is already registered");
	}
	
	output_map_[output->GetName()] = output;
}

bool OutputManager::WriteFile(const std::string& filename)
{
	OutputManagerWriter writer;
	return writer.Write(*this, filename);
}

void OutputManager::LoadGTOutputsFromSLAMFile(io::SLAMFile* file)
{
	LoadGTOutputsFromSLAMFile(file->Sensors, file, true);
}

slambench::outputs::Output *createGTOutput(const slambench::io::Sensor* sensor) {
	assert(sensor->IsGroundTruth());
	if(sensor->GetType() == slambench::io::GroundTruthSensor::kGroundTruthTrajectoryType) {
		return new slambench::outputs::Output("Trajectory", slambench::values::VT_POSE, true);
	} else if(sensor->GetType() == slambench::io::PointCloudSensor::kPointCloudType) {
		return new slambench::outputs::Output("PointCloud", slambench::values::VT_POINTCLOUD, true);
	} else {
		// unknown GT type
		assert(false);
	}
	throw std::logic_error("Unrecognised ground truth sensor type");
}

void OutputManager::LoadGTOutputsFromSLAMFile(io::SensorCollection& sensors, io::FrameCollection* gt_frames, bool with_point_cloud) {

	std::map<slambench::io::Sensor *, slambench::outputs::Output*> gt_outputs;
	
	// Initialise ground truth outputs
	for(slambench::io::Sensor *sensor : sensors) {
		if(sensor->IsGroundTruth()) {
			// we've found a GT sensor so try and create a GT output to represent it
			auto output = createGTOutput(sensor);
			output->SetActive(true);
			
			gt_outputs[sensor] = output;
			RegisterOutput(output);
		}
	}
		
	for(unsigned frame_idx = 0; frame_idx < gt_frames->GetFrameCount(); ++frame_idx) {

		auto i = gt_frames->GetFrame(frame_idx);
		if(!i->FrameSensor->IsGroundTruth()) {
			continue;
		}

		auto output = gt_outputs.at(i->FrameSensor);

		if(i->FrameSensor->GetType() == slambench::io::GroundTruthSensor::kGroundTruthTrajectoryType) {
			Eigen::Matrix4f K;
			memcpy(K.data(), i->GetData(), i->GetSize());
			i->FreeData();
			
			output->AddPoint(i->Timestamp, new slambench::values::PoseValue(K));
		}

		if(with_point_cloud and i->FrameSensor->GetType() == slambench::io::PointCloudSensor::kPointCloudType) {
			
			slambench::io::PointCloud *pc = slambench::io::PointCloud::FromRaw((char*)i->GetData());
			i->FreeData();
			
			auto pcv = new slambench::values::PointCloudValue();
			for(auto p : pc->Get()) {
				pcv->AddPoint({p.x,p.y,p.z});
			}
			delete pc;
			output->AddPoint(i->Timestamp, pcv);
		}
	}
}

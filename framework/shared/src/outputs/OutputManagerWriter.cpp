/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/sensor/Sensor.h"
#include "io/sensor/GroundTruthSensor.h"
#include "io/sensor/PointCloudSensor.h"
#include "io/SLAMFrame.h"
#include "io/format/PointCloud.h"

#include "outputs/OutputManagerWriter.h"
#include "outputs/OutputManager.h"

using namespace slambench::outputs;

slambench::io::SLAMFile* OutputManagerWriter::GetFile(OutputManager& outman)
{
	auto file = new slambench::io::SLAMFile();
	std::lock_guard<FastLock> lock (outman.GetLock());
	(void)lock; // I hate this werror
	
	if(!CreateSensors(outman, *file)) {
		delete file;
		return nullptr;
	}
	if(!CreateFrames(outman, *file)) {
		delete file;
		return nullptr;
	}
	
	return file;
}


bool OutputManagerWriter::Write(OutputManager& outman, const std::string& filename)
{
	auto file = GetFile(outman);
	if(!SerialiseFile(*file, filename)) {
		return false;
	}
	delete file;
	return true;
}

bool OutputManagerWriter::CreateSensors(OutputManager& outman, slambench::io::SLAMFile& file)
{
	for(auto output : outman) {
		if(output.second->IsActive()) {
			auto sensor = CreateSensor(output.second);
			if(sensor == nullptr) {
				continue;
			}
			output_map_.insert({output.second, sensor});
			file.Sensors.AddSensor(sensor);
		}
	}
	return true;
}

slambench::io::Sensor* OutputManagerWriter::CreateSensor(const BaseOutput* output)
{
	switch(output->GetType()) {
		case slambench::values::VT_POSE:
			return CreatePoseSensor(output);
		case slambench::values::VT_POINTCLOUD:
			return CreatePointCloudSensor(output);
		default:
			return nullptr;
	}
	__builtin_unreachable();
}

slambench::io::Sensor* OutputManagerWriter::CreatePoseSensor(const BaseOutput* output)
{
	// This is not really a ground truth. We need a 'trajectory' sensor
	auto sensor = new slambench::io::GroundTruthSensor("Output Trajectory");
	sensor->Description = output->GetName();
	
	return sensor;
}

slambench::io::Sensor* OutputManagerWriter::CreatePointCloudSensor(const BaseOutput* output)
{
	auto sensor = new slambench::io::PointCloudSensor("Output Point Cloud");
	sensor->Description = output->GetName();

	return sensor;
}

bool OutputManagerWriter::CreateFrames(OutputManager& outman, slambench::io::SLAMFile& file)
{
	for(auto output : outman) {
		slambench::outputs::BaseOutput *op = output.second;
		if(op->IsActive()) {
			for(auto frame : op->GetValues()) {
				auto output_frame = CreateFrame(op, frame.first, frame.second);
				if(output_frame != nullptr) {
					file.AddFrame(output_frame);
				}
			}
		}
	}
	
	return true;
}

slambench::io::SLAMFrame* OutputManagerWriter::CreateFrame(BaseOutput *output, TimeStamp ts, const slambench::values::Value* value)
{
	switch(value->GetType()) {
		case slambench::values::VT_POSE:
			return CreatePoseFrame(output, ts, (const slambench::values::PoseValue*)value);
		case slambench::values::VT_POINTCLOUD:
			return CreatePointCloudFrame(output, ts, (const slambench::values::PointCloudValue*)value);
		default:
			return nullptr;
	}
	__builtin_unreachable();
}

slambench::io::SLAMFrame* OutputManagerWriter::CreatePointCloudFrame(BaseOutput *output, TimeStamp ts, const slambench::values::PointCloudValue* value)
{
	auto pointcloud = new slambench::io::PointCloud();
	
	for(auto i : value->GetPoints()) {
		pointcloud->Get().push_back(slambench::io::Point(i.X, i.Y, i.Z));
	}
	
	auto frame = new slambench::io::SLAMInMemoryFrame();
	
	auto pointclouddata = pointcloud->ToRaw();
	
	frame->FrameSensor = output_map_.at(output);
	frame->Timestamp = ts;
	frame->SetData(pointclouddata.data(), pointclouddata.size());
	frame->SetVariableSize(pointclouddata.size());
	
	return frame;
}

slambench::io::SLAMFrame* OutputManagerWriter::CreatePoseFrame(BaseOutput *output, TimeStamp ts, const slambench::values::PoseValue* value)
{
	auto frame = new slambench::io::SLAMInMemoryFrame();
	frame->FrameSensor = output_map_.at(output);
	frame->Timestamp = ts;
	frame->SetData((void*)&value->GetValue());

	return frame;
}


bool OutputManagerWriter::SerialiseFile(slambench::io::SLAMFile& file, const std::string& filename)
{
	FILE *f = fopen(filename.c_str(), "w");
	slambench::io::SLAMFileSerialiser sfs(f);
	sfs.Serialise(file);
	fclose(f);
	
	return true;
}

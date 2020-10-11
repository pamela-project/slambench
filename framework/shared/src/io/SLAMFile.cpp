/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "io/SLAMFile.h"
#include "io/deserialisation/SLAMFileDeserialiser.h"
#include "io/serialisation/SLAMFileSerialiser.h"
#include "io/FrameBufferSource.h"
#include "io/sensor/Sensor.h"
#include "io/SLAMFrame.h"

#include <string>

using namespace slambench::io;

const SLAMFile::magic_num_t SLAMFile::MagicNum = "SLAM";

SLAMFrame* SLAMFile::GetFrame(unsigned int index) {
	return frames_.at(index);
}

unsigned int SLAMFile::GetFrameCount() {
	return frames_.size();
}

void SLAMFile::AddFrame(SLAMFrame* frame) {
	// Order ground truth frames before regular frames
	if(frame->FrameSensor->IsGroundTruth()) {
		AddGroundTruthFrame(frame);
	} else {
		AddRegularFrame(frame);
	}
}

void SLAMFile::AddRegularFrame(SLAMFrame *frame) {
	assert(!frame->FrameSensor->IsGroundTruth());
	
	// Place frame in correct place given the timestamp
	for (auto it = frames_.begin() ; it != frames_.end() ; it++) {
	  if((*it)->FrameSensor->IsGroundTruth()) { continue; }
	  if ((*it)->Timestamp <= frame->Timestamp) { continue; }
	  else {frames_.insert(it, frame);return;}
	}

	frames_.push_back(frame);
}

void SLAMFile::AddGroundTruthFrame(SLAMFrame* frame)
{
  	assert(frame->FrameSensor->IsGroundTruth());
	
	for (auto it = frames_.begin() ; it != frames_.end() ; it++) {
	  if(!(*it)->FrameSensor->IsGroundTruth()) { frames_.insert(it, frame);return; }
	  if ((*it)->Timestamp <= frame->Timestamp) { continue; }
	  else {frames_.insert(it, frame);return;}
	}

	frames_.push_back(frame);
}


Sensor* SLAMFile::GetSensor(const Sensor::sensor_type_t &type) {
	for(auto &i : Sensors) {
		if(i->GetType() == type) return i;
	}
	return nullptr;
}

SLAMFile* SLAMFile::Read(const std::string& filename, FrameBufferSource &fb_source) {
	FILE *base_file = fopen(filename.c_str(), "r");
	if(!base_file) {
		return nullptr;
	}
	
	SLAMFileDeserialiser deserialiser(base_file, &fb_source);
	SLAMFile *file = new SLAMFile();
	if(!deserialiser.Deserialise(*file)) {
		fclose(base_file);
		delete file;
		return nullptr;
	}
	
	return file;
}

bool SLAMFile::Write(const std::string& filename, SLAMFile& file, SLAMFileSerialiser::frame_callback_t callback) {
	FILE *base_file = fopen(filename.c_str(), "w");
	if(!base_file) {
		return false;
	}
	
	SLAMFileSerialiser serialiser(base_file);
	serialiser.FrameCallback = callback;
	if(!serialiser.Serialise(file)) {
		fclose(base_file);
		return false;
	}
	
	fclose(base_file);
	return true;
}

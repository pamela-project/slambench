/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/serialisation/SLAMFrameSerialiser.h"

#include "io/SLAMFrame.h"
#include "io/sensor/Sensor.h"

using namespace slambench::io;

SLAMFrameSerialiser::SLAMFrameSerialiser(FILE *target) : Serialiser(target) {
	
}

bool SLAMFrameSerialiser::Serialise(SLAMFrame &frame) {
	void *frame_data = frame.GetData();
	if(frame_data == nullptr) {
		printf("Could not get frame data\n");
		return false;
	}
	struct {
		decltype(frame.Timestamp) timestamp;
		decltype(frame.FrameSensor->Index) index;
	} __attribute__((packed)) data;
	
	data.timestamp = frame.Timestamp;
	data.index = frame.FrameSensor->Index;
	
	Write(&data, sizeof(data));
	if(frame.FrameSensor->IsVariableSize()) {
		uint32_t data_size = frame.GetSize();
		Write(&data_size, sizeof(data_size));
	}
	Write(frame_data, frame.GetSize());
	
	frame.FreeData();
	
	return true;
}

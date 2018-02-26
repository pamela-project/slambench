/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "io/deserialisation/SLAMFrameDeserialiser.h"
#include "io/FrameBufferSource.h"
#include "io/SLAMFrame.h"
#include "io/sensor/SensorCollection.h"
#include "io/sensor/Sensor.h"
#include "io/deserialisation/SLAMFileHeaderDeserialiser.h"

using namespace slambench::io;

SLAMFrameDeserialiser::SLAMFrameDeserialiser(FILE* file, SensorCollection& sensors, FrameBufferSource *fb_source) : Deserialiser(file), _sensors(sensors), _fb_source(fb_source) {

}

SLAMFrame* SLAMFrameDeserialiser::GetNextFrame() {
	DeserialisedFrame *dsf = new DeserialisedFrame(*_fb_source->Next(), File());
	
	if(!Read(&dsf->Timestamp, sizeof(dsf->Timestamp))) {
		delete dsf;
		return nullptr;
	}
	
	uint8_t sensor_index = 0;
	Read(&sensor_index, sizeof(sensor_index));
	dsf->FrameSensor = &_sensors.at(sensor_index);
	if(dsf->FrameSensor->IsVariableSize()) {
		uint32_t framesize;
		Read(&framesize, sizeof(framesize));
		dsf->SetVariableSize(framesize);
	}
	
	dsf->SetOffset(Offset());
	Skip(dsf->GetSize());
	
	return dsf;
}

bool SLAMFrameDeserialiser::HasNextFrame() {
	return Good();
}

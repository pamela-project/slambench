/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/openni15/ONI15Frame.h"
#include <iostream>

using namespace slambench::io::openni15;

ONI15Frame::ONI15Frame(Sensor * sensor, const xn::OutputMetaData * md) {

	FrameSensor = sensor;
	Timestamp.Ns = md->Timestamp() % 1000000000;
	Timestamp.S = md->Timestamp() / 1000000000;

	size_t size =  md->DataSize();
	_data = malloc(size);
    const XnUInt8* ptr = md->Data();
    if (ptr) {
          memcpy(_data,ptr,size);
      }
}

ONI15Frame::~ONI15Frame() {
	FreeData();
}

void* ONI15Frame::GetData() {
	assert(_data);
	return _data;
}

void ONI15Frame::FreeData() {
	free(_data);
	_data = nullptr;
}

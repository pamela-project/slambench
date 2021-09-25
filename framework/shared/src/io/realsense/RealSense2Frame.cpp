/*

 Copyright (c) 2020 University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/realsense/RealSense2Frame.h"

using namespace slambench::io::realsense;

RealSense2Frame::RealSense2Frame(Sensor *sensor, const rs2::frame& frameref) {
	FrameSensor = sensor;
	Timestamp.Ns = (int)frameref.get_timestamp() % 1000000000;
	Timestamp.S = frameref.get_timestamp() / 1000000000;
    data_ = malloc(frameref.get_data_size());
	memcpy(data_, frameref.get_data(), frameref.get_data_size());
}

void* RealSense2Frame::GetData() {
	return data_;
}

void RealSense2Frame::FreeData() {
	free(data_);
    data_ = nullptr;
}

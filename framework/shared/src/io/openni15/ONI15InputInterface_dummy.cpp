/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "io/openni15/ONI15InputInterface.h"
#include "io/sensor/CameraSensor.h"
#include "io/sensor/SensorCollection.h"
#include "io/openni15/ONI15FrameStream.h"

#include <iostream>

using namespace slambench::io;
using namespace slambench::io::openni15;

ONI15InputInterface::ONI15InputInterface() : _stream(nullptr), _sensors_ready(false) {
	std::cerr << "OpenNI2 has not been compiled." << std::endl;
	exit(1);
}


FrameStream& ONI15InputInterface::GetFrames() {
}

SensorCollection& ONI15InputInterface::GetSensors() {
}

void ONI15InputInterface::BuildSensors() {
}

void ONI15InputInterface::BuildStream() {
}

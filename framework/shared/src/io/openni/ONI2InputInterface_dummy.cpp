/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "io/openni2/ONI2InputInterface.h"
#include "io/sensor/CameraSensor.h"
#include "io/sensor/SensorCollection.h"
#include "io/openni2/ONI2FrameStream.h"

#include <iostream>

using namespace slambench::io;
using namespace slambench::io::openni2;

ONI2InputInterface::ONI2InputInterface() : _stream(nullptr), _sensors_ready(false) {
	std::cerr << "OpenNI2 has not been compiled." << std::endl;
	exit(1);
}

ONI2InputInterface::ONI2InputInterface(std::string ) : _stream(nullptr), _sensors_ready(false) {
	std::cerr << "OpenNI2 has not been compiled." << std::endl;
	exit(1);
}

FrameStream& ONI2InputInterface::GetFrames() {
}

SensorCollection& ONI2InputInterface::GetSensors() {
}

void ONI2InputInterface::BuildSensors() {	
}

void ONI2InputInterface::BuildStream() {
}

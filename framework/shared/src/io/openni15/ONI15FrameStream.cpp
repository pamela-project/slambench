/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/openni15/ONI15FrameStream.h"
#include "io/openni15/ONI15Frame.h"
#include "io/sensor/CameraSensor.h"
#include "io/sensor/DepthSensor.h"
#include "io/sensor/Sensor.h"

#include <iostream>
#include <stdexcept>

using namespace slambench::io::openni15;
using namespace slambench::io;

ONI15FrameStream::ONI15FrameStream(xn::Context *context) : _context(context) {
}

SLAMFrame* ONI15FrameStream::GetNextFrame() {

	ONI15Frame  *frame  = nullptr;

	_context->WaitAnyUpdateAll();

	// all frame seen, we clean them all.
	if (DMD and IMD) {
		delete DMD; DMD = nullptr;
		delete IMD; IMD = nullptr;
	}


	// no Depth we give it a try
	if (!DMD) {
		DMD = new xn::DepthMetaData();
		_depth_generator->GetMetaData(*DMD);
		if (DMD->DataSize() == 0) {
			delete DMD; DMD = nullptr;
		} else {
			frame = new ONI15Frame(_sensor_map.at(_depth_generator), DMD);
			return frame;
		}
	}

	// no image we give it a try
	if (!IMD) {
		IMD = new xn::ImageMetaData();
		_image_generator->GetMetaData(*IMD);
		if (IMD->DataSize() == 0) {
			delete IMD; IMD = nullptr;
		} else {
			frame = new ONI15Frame(_sensor_map.at(_image_generator), IMD);
			return frame;
		}
	}

	// No new frame found we have a problem !
	assert(false);
	return nullptr;

}

bool ONI15FrameStream::HasNextFrame() {
	return true;

}


bool ONI15FrameStream::ActivateSensor(CameraSensor* sensor) {

	assert(_context);

	if(sensor->GetType()  == CameraSensor::kCameraType) {
		xn::ImageGenerator *stream = new xn::ImageGenerator();
		if (stream->Create(*_context) != XN_STATUS_OK) {
			std::cout << ( "OpenNI15: Create(_context) ERROR  !!!") << std::endl;
			return false;
		}

		_image_generator = stream;
		_sensor_map[stream] = sensor;
	}else if(sensor->GetType()  == DepthSensor::kDepthType) {
		xn::DepthGenerator *stream  = new xn::DepthGenerator();
		if (stream->Create(*_context) != XN_STATUS_OK) {
			std::cout << ( "OpenNI15: Create(_context) ERROR  !!!") << std::endl;
			return false;
		}

		_depth_generator = stream;
		_sensor_map[stream] = sensor;
	}else{
		throw std::logic_error("Unrecognized sensor type");
	}



	return true;
}

bool ONI15FrameStream::StartStreams() {

	if (_context->StartGeneratingAll() != XN_STATUS_OK) {
		std::cout << ( "OpenNI15 _context.StartGeneratingAll() ERROR !!!") << std::endl;
		return false;
	}
	return true;
}

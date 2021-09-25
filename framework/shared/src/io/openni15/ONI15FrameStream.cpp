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

ONI15FrameStream::ONI15FrameStream(xn::Context *context) : context_(context) {
}

SLAMFrame* ONI15FrameStream::GetNextFrame() {

	ONI15Frame  *frame  = nullptr;

	context_->WaitAnyUpdateAll();

	// all frame seen, we clean them all.
	if (DMD_ and IMD_) {
		delete DMD_; DMD_ = nullptr;
		delete IMD_; IMD_ = nullptr;
	}


	// no Depth we give it a try
	if (!DMD_) {
		DMD_ = new xn::DepthMetaData();
		depth_generator_->GetMetaData(*DMD_);
		if (DMD_->DataSize() == 0) {
			delete DMD_; DMD_ = nullptr;
		} else {
			frame = new ONI15Frame(_sensor_map.at(depth_generator_), DMD_);
			return frame;
		}
	}

	// no image we give it a try
	if (!IMD_) {
		IMD_ = new xn::ImageMetaData();
		image_generator_->GetMetaData(*IMD_);
		if (IMD_->DataSize() == 0) {
			delete IMD_; IMD_ = nullptr;
		} else {
			frame = new ONI15Frame(_sensor_map.at(image_generator_), IMD_);
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

	assert(context_);

	if(sensor->GetType()  == CameraSensor::kCameraType) {
		xn::ImageGenerator *stream = new xn::ImageGenerator();
		if (stream->Create(*context_) != XN_STATUS_OK) {
			std::cout << ( "OpenNI15: Create(context_) ERROR  !!!") << std::endl;
			return false;
		}

		image_generator_ = stream;
		_sensor_map[stream] = sensor;
	}else if(sensor->GetType()  == DepthSensor::kDepthType) {
		xn::DepthGenerator *stream  = new xn::DepthGenerator();
		if (stream->Create(*context_) != XN_STATUS_OK) {
			std::cout << ( "OpenNI15: Create(context_) ERROR  !!!") << std::endl;
			return false;
		}

		depth_generator_ = stream;
		_sensor_map[stream] = sensor;
	}else{
		throw std::logic_error("Unrecognized sensor type");
	}



	return true;
}

bool ONI15FrameStream::StartStreams() {

	if (context_->StartGeneratingAll() != XN_STATUS_OK) {
		std::cout << ( "OpenNI15 context_.StartGeneratingAll() ERROR !!!") << std::endl;
		return false;
	}
	return true;
}

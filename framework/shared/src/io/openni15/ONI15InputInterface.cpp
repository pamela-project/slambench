/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "io/openni15/ONI15InputInterface.h"
#include "io/sensor/CameraSensor.h"
#include "io/sensor/DepthSensor.h"
#include "io/sensor/SensorCollection.h"
#include "io/openni15/ONI15FrameStream.h"

#include <iostream>

#define linux true

#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>




using namespace slambench::io;
using namespace slambench::io::openni15;

ONI15InputInterface::ONI15InputInterface() : _stream(nullptr), _sensors_ready(false) {


	this->_context = new xn::Context();
    if (_context->Init() != XN_STATUS_OK) {
    	std::cerr << ( "OpenNI15: Error  Init Failed.") << std::endl;
    	exit(1);
    	return;
    }


    if (_context->StartGeneratingAll() != XN_STATUS_OK) {
    	std::cerr << ( "OpenNI15: _context.StartGeneratingAll() ERROR !!!") << std::endl;
    	exit(1);
    	return;
    }


    std::cerr << ( "OpenNI15:  Ready !!!") << std::endl;




}


FrameStream& ONI15InputInterface::GetFrames() {

	if(_stream == nullptr) BuildStream();
	
	return *_stream;
}





SensorCollection& ONI15InputInterface::GetSensors() {

	BuildSensors();
	return _sensors;
}



CameraSensor *ONI15InputInterface::BuildCameraSensor() {
	
	std::string sensor_type =  slambench::io::CameraSensor::kCameraType;

	CameraSensor *sensor = new CameraSensor(sensor_type);
	sensor->Description = "ONI15 RGB Sensor";
	sensor->FrameFormat = frameformat::Raster;

	sensor->PixelFormat = pixelformat::RGB_III_888;
	sensor->Width = 640;
	sensor->Height = 480;

	sensor->Pose = Eigen::Matrix4f::Identity();

	bzero(sensor->Intrinsics, sizeof(sensor->Intrinsics));
	// guess the intrinsics for now
	std::cerr << "I'm guessing which intrinsics to use" << std::endl;
	sensor->Intrinsics[0] = 0.751875;
	sensor->Intrinsics[1] = 1.0;
	sensor->Intrinsics[2] = 0.5;
	sensor->Intrinsics[3] = 0.5;
	
	std::cerr << "Built an RGB  sensor with " << sensor->Width << ", " << sensor->Height << ", " << sensor->GetFrameSize(NULL) << "b per frame" << std::endl;

	return sensor;
}



DepthSensor *ONI15InputInterface::BuildDepthSensor() {

	std::string sensor_type =  slambench::io::DepthSensor::kDepthType;

	DepthSensor *sensor = new DepthSensor(sensor_type);
	sensor->Description = "ONI15 Depth Sensor";
	sensor->FrameFormat = frameformat::Raster;

	// find a suitable video mode...?
	sensor->PixelFormat = pixelformat::D_I_16;

	sensor->Width = 640;
	sensor->Height = 480;

	sensor->Pose = Eigen::Matrix4f::Identity();

	bzero(sensor->Intrinsics, sizeof(sensor->Intrinsics));
	// guess the intrinsics for now
	std::cerr << "I'm guessing which intrinsics to use" << std::endl;
	sensor->Intrinsics[0] = 0.751875;
	sensor->Intrinsics[1] = 1.0;
	sensor->Intrinsics[2] = 0.5;
	sensor->Intrinsics[3] = 0.5;
	
	std::cerr << "Built a depth sensor with " << sensor->Width << ", " << sensor->Height << ", " << sensor->GetFrameSize(NULL) << "b per frame" << std::endl;
	
	return sensor;
}

void ONI15InputInterface::BuildSensors() {

	if(_sensors_ready) return;


	auto depth_sensor = BuildDepthSensor();
	depth_sensor->Index = _sensors.size();
	_sensors.AddSensor(depth_sensor);

	auto rgb_sensor = BuildCameraSensor();
	rgb_sensor->Index = _sensors.size();
	_sensors.AddSensor(rgb_sensor);

	_sensors_ready = true;
}

void ONI15InputInterface::BuildStream() {

	BuildSensors();
	_stream = new ONI15FrameStream(_context);

	bool depth_found = false;
	bool rgb_found= false;



	for(auto *sensor : _sensors) {
		if(sensor->GetType() == slambench::io::CameraSensor::kCameraType) {
			rgb_found = true;
			assert(_stream->ActivateSensor((CameraSensor*)sensor));
		} else if (sensor->GetType() == slambench::io::DepthSensor::kDepthType) {
			depth_found = true;
			assert(_stream->ActivateSensor((DepthSensor*)sensor));
		}
	}
	assert(rgb_found and depth_found);
	_stream->StartStreams();
}

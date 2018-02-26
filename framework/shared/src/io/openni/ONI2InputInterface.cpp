/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "io/openni2/ONI2InputInterface.h"
#include "OpenNI.h"
#include "io/sensor/CameraSensor.h"
#include "io/sensor/DepthSensor.h"
#include "io/sensor/SensorCollection.h"
#include "io/openni2/ONI2FrameStream.h"

#include <iostream>

using namespace slambench::io;
using namespace slambench::io::openni2;

ONI2InputInterface::ONI2InputInterface() : _stream(nullptr), _sensors_ready(false) {

	openni::OpenNI::initialize();
	
	_device = new openni::Device();
	openni::Status status = _device->open(openni::ANY_DEVICE);
	if (status == openni::STATUS_OK) {
	std::cout << " ** OpenNI2 Device " <<  _device->getDeviceInfo().getUsbProductId() <<  ":" <<  _device->getDeviceInfo().getUsbVendorId()
			<< " " << _device->getDeviceInfo().getName() << ", "<< _device->getDeviceInfo().getVendor() << std::endl;
	} else {
		std::cout << " ** OpenNI2 Error with the device Status:" ;
		switch (status) {
				case openni::STATUS_OK :  std::cout << "STATUS_OK" ; break;
				case openni::STATUS_ERROR :  std::cout << "STATUS_ERROR" ; break;
				case openni::STATUS_NOT_IMPLEMENTED :  std::cout << "STATUS_NOT_IMPLEMENTED" ; break;
				case openni::STATUS_NOT_SUPPORTED :  std::cout << "STATUS_NOT_SUPPORTED" ; break;
				case openni::STATUS_BAD_PARAMETER :  std::cout << "STATUS_BAD_PARAMETER" ; break;
				case openni::STATUS_OUT_OF_FLOW :  std::cout << "STATUS_OUT_OF_FLOW" ; break;
				case openni::STATUS_NO_DEVICE :  std::cout << "STATUS_NO_DEVICE" ; break;
				case openni::STATUS_TIME_OUT :  std::cout << "STATUS_TIME_OUT" ; break;
		}
		std::cout << std::endl;
		exit(1);
	}

}

ONI2InputInterface::ONI2InputInterface(std::string oni2_filename) : _stream(nullptr), _sensors_ready(false) {
	openni::OpenNI::initialize();
	
	_device = new openni::Device();
	openni::Status status = _device->open(oni2_filename.c_str());

	if (status != openni::STATUS_OK) {
		std::cout << " ** OpenNI2 Error with the device Status:" ;
		switch (status) {
				case openni::STATUS_OK :  std::cout << "STATUS_OK" ; break;
				case openni::STATUS_ERROR :  std::cout << "STATUS_ERROR" ; break;
				case openni::STATUS_NOT_IMPLEMENTED :  std::cout << "STATUS_NOT_IMPLEMENTED" ; break;
				case openni::STATUS_NOT_SUPPORTED :  std::cout << "STATUS_NOT_SUPPORTED" ; break;
				case openni::STATUS_BAD_PARAMETER :  std::cout << "STATUS_BAD_PARAMETER" ; break;
				case openni::STATUS_OUT_OF_FLOW :  std::cout << "STATUS_OUT_OF_FLOW" ; break;
				case openni::STATUS_NO_DEVICE :  std::cout << "STATUS_NO_DEVICE" ; break;
				case openni::STATUS_TIME_OUT :  std::cout << "STATUS_TIME_OUT" ; break;
		}
		std::cout << std::endl;
		exit(1);
	}

}

FrameStream& ONI2InputInterface::GetFrames() {
	if(_stream == nullptr) BuildStream();
	
	return *_stream;
}

SensorCollection& ONI2InputInterface::GetSensors() {
	BuildSensors();
	return _sensors;
}



CameraSensor *ONI2InputInterface::BuildCameraSensor(const openni::SensorInfo *sensor_info) {

	assert (sensor_info->getSensorType() == openni::SENSOR_COLOR);
	
	std::string sensor_type =  slambench::io::CameraSensor::kCameraType;

	CameraSensor *sensor = new CameraSensor(sensor_type);
	sensor->Description = "ONI2 RGB Sensor";
	sensor->FrameFormat = frameformat::Raster;

	// find a suitable video mode...?
	auto vid_mode = sensor_info->getSupportedVideoModes()[0];
	switch(vid_mode.getPixelFormat()) {
		case openni::PIXEL_FORMAT_RGB888: sensor->PixelFormat = pixelformat::RGB_III_888; break;
		case openni::PIXEL_FORMAT_DEPTH_1_MM: sensor->PixelFormat = pixelformat::D_I_16; break;
		default:
			throw std::logic_error("Unknown pixel format");
	}

	sensor->Width = vid_mode.getResolutionX();
	sensor->Height = vid_mode.getResolutionY();

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



DepthSensor *ONI2InputInterface::BuildDepthSensor(const openni::SensorInfo *sensor_info) {

	assert (sensor_info->getSensorType() == openni::SENSOR_DEPTH);

	std::string sensor_type =  slambench::io::DepthSensor::kDepthType;

	DepthSensor *sensor = new DepthSensor(sensor_type);
	sensor->Description = "ONI2 Depth Sensor";
	sensor->FrameFormat = frameformat::Raster;

	// find a suitable video mode...?
	auto vid_mode = sensor_info->getSupportedVideoModes()[0];
	switch(vid_mode.getPixelFormat()) {
		case openni::PIXEL_FORMAT_RGB888: sensor->PixelFormat = pixelformat::RGB_III_888; break;
		case openni::PIXEL_FORMAT_DEPTH_1_MM: sensor->PixelFormat = pixelformat::D_I_16; break;
		default:
			throw std::logic_error("Unknown pixel format");
	}

	sensor->Width = vid_mode.getResolutionX();
	sensor->Height = vid_mode.getResolutionY();

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

void ONI2InputInterface::BuildSensors() {	
	if(_sensors_ready) return;

	auto depth_sensor_info = _device->getSensorInfo(openni::SENSOR_DEPTH);
	if(depth_sensor_info != nullptr) {
		auto sensor = BuildDepthSensor(depth_sensor_info);
		sensor->Index = _sensors.size();
		_sensors.AddSensor(sensor);
	}
	

	auto color_sensor_info = _device->getSensorInfo(openni::SENSOR_COLOR);
	if(color_sensor_info != nullptr) {
		auto sensor = BuildCameraSensor(color_sensor_info);
		sensor->Index = _sensors.size();
		_sensors.AddSensor(sensor);
	}
	


	_sensors_ready = true;
}

void ONI2InputInterface::BuildStream() {
	BuildSensors();
	_stream = new ONI2FrameStream(_device);

	bool depth_found = false;
	bool rgb_found= false;

	for(auto *sensor : _sensors) {
		if(sensor->GetType() == slambench::io::CameraSensor::kCameraType) {
			rgb_found = true;
			_stream->ActivateSensor((CameraSensor*)sensor);
		} else if (sensor->GetType() == slambench::io::DepthSensor::kDepthType) {
			depth_found = true;
			_stream->ActivateSensor((DepthSensor*)sensor);
		}
	}
	assert(rgb_found and depth_found);
	_stream->StartStreams();
}

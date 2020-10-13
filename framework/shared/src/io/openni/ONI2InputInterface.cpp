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
#include <io/sensor/sensor_builder.h>

using namespace slambench::io;
using namespace slambench::io::openni2;

ONI2InputInterface::ONI2InputInterface() : stream_(nullptr), sensors_ready_(false) {

	openni::OpenNI::initialize();

    device_ = new openni::Device();
	openni::Status status = device_->open(openni::ANY_DEVICE);
	if (status == openni::STATUS_OK) {
	std::cout << " ** OpenNI2 Device " << device_->getDeviceInfo().getUsbProductId() << ":" << device_->getDeviceInfo().getUsbVendorId()
              << " " << device_->getDeviceInfo().getName() << ", " << device_->getDeviceInfo().getVendor() << std::endl;
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

ONI2InputInterface::ONI2InputInterface(const std::string& oni2_filename) : stream_(nullptr), sensors_ready_(false) {
	openni::OpenNI::initialize();

    device_ = new openni::Device();
	openni::Status status = device_->open(oni2_filename.c_str());

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
	if(stream_ == nullptr) BuildStream();
	
	return *stream_;
}

SensorCollection& ONI2InputInterface::GetSensors() {
	BuildSensors();
	return sensors_;
}

void ONI2InputInterface::BuildSensors() {
	if(sensors_ready_) return;

	auto depth_sensor_info = device_->getSensorInfo(openni::SENSOR_DEPTH);
	if(depth_sensor_info != nullptr) {
        // find a suitable video mode...?
        auto vid_mode = depth_sensor_info->getSupportedVideoModes()[0];
        slambench::io::CameraSensor::intrinsics_t intrinsics;
        // guess the intrinsics for now
        std::cerr << "I'm guessing which intrinsics to use" << std::endl;
        intrinsics[0] = 0.751875;
        intrinsics[1] = 1.0;
        intrinsics[2] = 0.5;
        intrinsics[3] = 0.5;

        auto depth_sensor = DepthSensorBuilder()
                .name("Depth")
                .description("ONI2 Depth Sensor")
                .size(vid_mode.getResolutionX(), vid_mode.getResolutionY())
                .pose(Eigen::Matrix4f::Identity())
                .intrinsics(intrinsics)
                .frameFormat(frameformat::Raster)
                .pixelFormat(pixelformat::D_I_16)
                .rate(vid_mode.getFps())
                //.distortion(distortion_type, distortion)
                //.disparity(disparity_type, disparity_params)
                .index(sensors_.size())
                .build();

		sensors_.AddSensor(depth_sensor);
	}

	auto color_sensor_info = device_->getSensorInfo(openni::SENSOR_COLOR);
	if(color_sensor_info != nullptr) {
		// find a suitable video mode...?
        auto vid_mode = color_sensor_info->getSupportedVideoModes()[0];

        // guess the intrinsics for now
        slambench::io::CameraSensor::intrinsics_t intrinsics;
        intrinsics[0] = 0.751875;
        intrinsics[1] = 1.0;
        intrinsics[2] = 0.5;
        intrinsics[3] = 0.5;

        auto sensor = CameraSensorBuilder()
                .name("RGB")
                .description("ONI2 RGB Sensor")
                .rate(vid_mode.getFps())
                .size(vid_mode.getResolutionX(), vid_mode.getResolutionY())
                .pose(Eigen::Matrix4f::Identity())
                .intrinsics(intrinsics)
                //.distortion(distortion_type, distortion)
                .frameFormat(frameformat::Raster)
                .pixelFormat(pixelformat::RGB_III_888)
                .index(sensors_.size())
                .build();
		sensors_.AddSensor(sensor);
	}

    sensors_ready_ = true;
}

void ONI2InputInterface::BuildStream() {
	BuildSensors();
    stream_ = new ONI2FrameStream(device_);

	bool depth_found = false;
	bool rgb_found= false;

	for(auto *sensor : sensors_) {
		if(sensor->GetType() == slambench::io::CameraSensor::kCameraType) {
			rgb_found = true;
			stream_->ActivateSensor((CameraSensor*)sensor);
		} else if (sensor->GetType() == slambench::io::DepthSensor::kDepthType) {
			depth_found = true;
			stream_->ActivateSensor((DepthSensor*)sensor);
		}
	}
	assert(rgb_found and depth_found);
	stream_->StartStreams();
}

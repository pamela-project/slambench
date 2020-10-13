/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/openni2/ONI2FrameStream.h"
#include "io/openni2/ONI2Frame.h"
#include "io/sensor/CameraSensor.h"
#include "io/sensor/DepthSensor.h"
#include "io/sensor/Sensor.h"

#include <iostream>
#include <stdexcept>

using namespace slambench::io::openni2;
using namespace slambench::io;

ONI2FrameStream::ONI2FrameStream(openni::Device *device) : device_(device) {}

SLAMFrame* ONI2FrameStream::GetNextFrame() {
    int ready_stream_idx = 0;

    auto status = openni::OpenNI::waitForAnyStream(streams_.data(), streams_.size(), &ready_stream_idx, openni::TIMEOUT_FOREVER);
    if(status != openni::STATUS_OK) {
        return nullptr;
    } else {
        openni::VideoFrameRef *ref = new openni::VideoFrameRef();
        openni::VideoStream *ready_stream = streams_.at(ready_stream_idx);
        ready_stream->readFrame(ref);
        auto frame = new ONI2Frame(sensor_map_.at(ready_stream), *ref);
        delete ref;

        return frame;
    }
}

bool ONI2FrameStream::HasNextFrame() {
    for(auto &i : streams_) {
        if(i->isValid()) return true;
    }
    return false;
}

class FrameAllocator : public openni::VideoStream::FrameAllocator {
public:
    FrameAllocator() : _ptr(malloc(0)),_size(0) {}

    void* allocateFrameBuffer(int size) override {
        if(size != _size) {
            _ptr = realloc(_ptr, size);
            _size = size;
        }

        return _ptr;
    }

    void freeFrameBuffer(void* data) override {
        if(data != _ptr) {
            throw std::logic_error("Attempted to free a pointer with the wrong allocator");
        }
    }

private:
    void *_ptr;
    int _size;
};

bool ONI2FrameStream::ActivateSensor(CameraSensor* sensor) {

    openni::VideoStream *stream = new openni::VideoStream();


    openni::SensorType sensor_type;
    if(sensor->GetType()  == CameraSensor::kCameraType)
        sensor_type = openni::SENSOR_COLOR;
    else if(sensor->GetType()  == DepthSensor::kDepthType)
        sensor_type = openni::SENSOR_DEPTH;
    else
        throw std::logic_error("Unrecognized sensor type");

    openni::Status nRetVal  =  stream->create(*device_, sensor_type);

    if (nRetVal != openni::STATUS_OK) {
        printf("Failed to create %s \n", openni::OpenNI::getExtendedError());
        exit(1);
    }

    openni::VideoMode vidmode;
    vidmode.setFps(30);
    openni::PixelFormat oniformat;
    switch(sensor->PixelFormat) {
        case pixelformat::RGB_III_888: oniformat = openni::PIXEL_FORMAT_RGB888; break;
        case pixelformat::D_I_16: oniformat = openni::PIXEL_FORMAT_DEPTH_1_MM; break;
        default:
            throw std::logic_error("Unknown pixel format");
    }
    vidmode.setPixelFormat(oniformat);
    vidmode.setResolution(sensor->Width, sensor->Height);

    stream->setVideoMode(vidmode);
    stream->setMirroringEnabled(false);

    stream->setFrameBuffersAllocator(new FrameAllocator());

    streams_.push_back(stream);
    sensor_map_[stream] = sensor;

    return true;
}

bool ONI2FrameStream::StartStreams() {
    //device_->setDepthColorSyncEnabled(false);
    device_->setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    //device_->setImageRegistrationMode(openni::IMAGE_REGISTRATION_OFF);

    for(auto i : streams_) {
        std::cerr << "Start stream ... ";
        auto nRetVal = i->start();
        if (nRetVal != openni::STATUS_OK) {
            std::cerr << "FAILED" << openni::OpenNI::getExtendedError()<< std::endl;
            exit(1);
        } else {
            std::cerr << "WORK" << std::endl;
        }
    }
    return true;
}

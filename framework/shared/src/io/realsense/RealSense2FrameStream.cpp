/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/realsense/RealSense2FrameStream.h"
#include "io/realsense/RealSense2Frame.h"
#include "io/sensor/CameraSensor.h"
#include "io/sensor/DepthSensor.h"
#include "io/sensor/Sensor.h"

#include <iostream>
#include <stdexcept>

using namespace slambench::io::realsense;
using namespace slambench::io;

RealSense2FrameStream::
RealSense2FrameStream(rs2::pipeline &pipe) : pipe_(pipe) {}

RealSense2FrameStream::~RealSense2FrameStream() {}

SLAMFrame* RealSense2FrameStream::GetNextFrame() {
    for(auto frame = new_frames_.begin(); frame < new_frames_.end() ; frame++)
    {
        auto it = sensor_map_.find(frame->get_profile().stream_type());
        if(it != sensor_map_.end())
        {   auto fr = new RealSense2Frame(it->second, *frame);
            new_frames_.erase(frame);
            return fr;
        }
    }
    return nullptr;
}

bool RealSense2FrameStream::ActivateSensor(rs2_stream stream, Sensor* sensor) {
    sensor_map_[stream] = sensor;
    return true;
}

bool RealSense2FrameStream::HasNextFrame() {
    if(!new_frames_.empty())
        return true;

    rs2::frameset frameset;
    pipe_.poll_for_frames(&frameset);
    for (const rs2::frame& f : frameset)
         new_frames_.push_back(f);

    return true;
}

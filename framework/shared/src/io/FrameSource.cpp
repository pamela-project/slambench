/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/FrameSource.h"
#include "io/SLAMFrame.h"
#include "io/sensor/Sensor.h"

#include <cassert>
#include <iostream>

using namespace slambench::io;

FrameCollection::~FrameCollection() {

}

FrameStream::~FrameStream() {

}

FrameCollectionStream::FrameCollectionStream(FrameCollection& base_collection) : _collection(base_collection), _index(0) {

}

SLAMFrame* FrameCollectionStream::GetNextFrame() {
	if(!HasNextFrame()) {
		return nullptr;
	}
	return _collection.GetFrame(_index++);
}

bool FrameCollectionStream::HasNextFrame() {
	return _index < _collection.GetFrameCount();
}

GTBufferingFrameStream::GTFrameCollection::GTFrameCollection(GTBufferingFrameStream& gt_stream) : gt_stream_(gt_stream)
{
	
}

GTBufferingFrameStream::GTFrameCollection::~GTFrameCollection()
{

}

SLAMFrame* GTBufferingFrameStream::GTFrameCollection::GetFrame(unsigned int index)
{
	return gt_stream_.gt_frames_.at(index);
}

unsigned int GTBufferingFrameStream::GTFrameCollection::GetFrameCount()
{
	return gt_stream_.gt_frames_.size();
}

GTBufferingFrameStream::GTBufferingFrameStream(FrameStream& base_stream) : base_stream_(base_stream), buffered_frame_(nullptr)
{
	fastForward();
}


GTBufferingFrameStream::~GTBufferingFrameStream()
{

}

SLAMFrame* GTBufferingFrameStream::GetNextFrame()
{
	if(buffered_frame_ != nullptr) {
		auto frame = buffered_frame_;
		buffered_frame_ = nullptr;
		return frame;
	}
	
	return base_stream_.GetNextFrame();
}

bool GTBufferingFrameStream::HasNextFrame()
{
	return base_stream_.HasNextFrame() || buffered_frame_ != nullptr;
}

FrameCollection* GTBufferingFrameStream::GetGTFrames()
{
	// TODO : memory leak here
	return new GTFrameCollection(*this);
}

void GTBufferingFrameStream::fastForward()
{
	assert(gt_frames_.empty());
	
	SLAMFrame *next_frame = nullptr;
	while(true) {
		if(!base_stream_.HasNextFrame()) {
			break;
		}
		next_frame = base_stream_.GetNextFrame();
		if(next_frame == nullptr) {
			break;
		}
		
		if(!next_frame->FrameSensor->IsGroundTruth()) {
			break;
		}
		
		gt_frames_.push_back(next_frame);
	}
	
	buffered_frame_ = next_frame;
}

RealTimeFrameStream::RealTimeFrameStream(FrameStream* base_stream, double multiplier, bool should_pause) : base_stream_(base_stream), acceleration_(multiplier), should_pause_(should_pause)
{
	
}

RealTimeFrameStream::~RealTimeFrameStream()
{

}

bool RealTimeFrameStream::HasNextFrame()
{
	return base_stream_->HasNextFrame();
}

SLAMFrame* RealTimeFrameStream::GetNextFrame()
{
	SLAMFrame *frame = nullptr;
	do {
		frame = base_stream_->GetNextFrame();
		if(frame) {
			frame->Timestamp *= acceleration_;
		}
	} while(HasNextFrame() && !is_valid_frame(frame));
	
	if(frame) {
		auto now = slambench::TimeStamp::Now();
		
		auto last_timestamp = last_frame_timestamp_[frame->FrameSensor];
		auto last_realtimestamp = last_frame_realtimestamp_[frame->FrameSensor];
		
		last_frame_timestamp_[frame->FrameSensor] = frame->Timestamp;
		last_frame_realtimestamp_[frame->FrameSensor] = now;
		
		// if the slam time since the last frame is greater than the real time
		// since the last frame, then sleep until it isn't.
		auto slam_time_since_last_frame = frame->Timestamp - last_timestamp;
		auto real_time_since_last_frame = now - last_realtimestamp;
		if(slam_time_since_last_frame > real_time_since_last_frame) {
			if(should_pause_) {
				slambench::Sleep(slam_time_since_last_frame - real_time_since_last_frame);
			}
		}
	}
	
	return frame;
}

bool RealTimeFrameStream::is_valid_frame(SLAMFrame* frame) const
{
	if(frame == nullptr) {
		throw std::logic_error("");
	}
	
	// first frame encountered for a sensor is always valid
	if(last_frame_timestamp_.count(frame->FrameSensor) == 0) {
		return true;
	}
	
	// frame is valid if the time between the frame and the last returned frame
	// is greater than the real time since the last returned frame
	slambench::TimeStamp last_sensor_time {0,0};
	if(last_frame_timestamp_.count(frame->FrameSensor)) {
		last_sensor_time = last_frame_timestamp_.at(frame->FrameSensor);
	}
	
	slambench::TimeStamp last_sensor_realtime {0,0};
	if(last_frame_realtimestamp_.count(frame->FrameSensor)) {
		last_sensor_realtime = last_frame_realtimestamp_.at(frame->FrameSensor);
	}
	
	auto time_since_last_frame = frame->Timestamp - last_sensor_time;	
	auto realtime_since_last_frame = slambench::TimeStamp::Now() - last_sensor_realtime;
	
	return time_since_last_frame >= realtime_since_last_frame;
	
}

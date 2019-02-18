/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/FrameBuffer.h"

#include <cstdlib>
#include <stdexcept>

using namespace slambench::io;

FrameBuffer::FrameBuffer() : _lock(false), _size(0), _data(nullptr) {
}

FrameBuffer::~FrameBuffer() {
	// This is dangerous if the frame buffer hasn't been released
	if(_data) {
		free(_data);
	}
}

void FrameBuffer::Acquire() {
	if(_lock.exchange(true)) {
		throw std::logic_error("FrameBuffer is already locked");
	}
}

bool FrameBuffer::TryAcquire() {
	if(_lock.exchange(true)) return false;
	return true;
}

void FrameBuffer::Release() {
	_lock = false;
}

void FrameBuffer::ResetBuffer() {
	if(_data) {
		free(_data);
		_data = malloc(0);
	}
}

bool FrameBuffer::Busy() {
	return _lock;
}

bool FrameBuffer::Reserve(size_t new_size) {
	if(new_size > _size) {
		_size = new_size;
	}
	
	if(_data) {
		_data = realloc(_data, _size);
		if(_data == nullptr) return false;
	}
	
	return true;
}

void *FrameBuffer::Data() {
	if(_data == nullptr) {
		_data = malloc(_size);
	}
	return _data;
}

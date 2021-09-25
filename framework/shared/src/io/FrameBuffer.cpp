/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/FrameBuffer.h"

#include <cstdlib>
#include <stdexcept>

using namespace slambench::io;

FrameBuffer::FrameBuffer() : lock_(false), size_(0), data_(nullptr) {
}

FrameBuffer::~FrameBuffer() {
	// This is dangerous if the frame buffer hasn't been released
	if(data_) {
		free(data_);
	}
}

void FrameBuffer::Acquire() {
	if(lock_.exchange(true)) {
		throw std::logic_error("FrameBuffer is already locked");
	}
}

bool FrameBuffer::TryAcquire() {
	if(lock_.exchange(true)) return false;
	return true;
}

void FrameBuffer::Release() {
    lock_ = false;
}

void FrameBuffer::ResetBuffer() {
	if(data_) {
		free(data_);
        data_ = malloc(0);
	}
}

bool FrameBuffer::Busy() {
	return lock_;
}

bool FrameBuffer::Reserve(size_t new_size) {
	if(new_size > size_) {
        size_ = new_size;
	}
	
	if(data_) {
        data_ = realloc(data_, size_);
		if(data_ == nullptr) return false;
	}
	
	return true;
}

void *FrameBuffer::Data() {
	if(data_ == nullptr) {
        data_ = malloc(size_);
	}
	return data_;
}

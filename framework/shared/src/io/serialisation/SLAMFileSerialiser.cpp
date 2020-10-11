/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/serialisation/SLAMFileSerialiser.h"
#include "io/serialisation/SLAMFileHeaderSerialiser.h"
#include "io/serialisation/SLAMFrameSerialiser.h"

#include "io/SLAMFile.h"

#include <stdexcept>
#include <iostream>

using namespace slambench::io;

SLAMFileSerialiser::SLAMFileSerialiser(FILE *target) : Serialiser(target), FrameCallback(nullptr) {
	
}

bool SLAMFileSerialiser::Serialise(SLAMFile &file) {
	if(!SerialiseHeader(file)) return false;
	if(!SerialiseFrames(file)) return false;
	return true;
}

bool SLAMFileSerialiser::SerialiseHeader(const SLAMFile &file) {
	SLAMFileHeaderSerialiser sfhs(File());
	
	return sfhs.Serialise(file);	
}

bool SLAMFileSerialiser::SerialiseFrames(SLAMFile &file) {
	SLAMFrameSerialiser sfs(File());
	
	uint32_t frame_idx = 0;
	
	for(unsigned int i = 0; i < file.GetFrameCount(); ++i) {
		SLAMFrame *frame = file.GetFrame(i);
		
		if(!sfs.Serialise(*frame)) {
			std::cerr<<"Failed to serialise frame";
			return false;
		}
		if(FrameCallback != nullptr) {
			FrameCallback(frame_idx, file.GetFrameCount());
		}
		frame_idx++;
	}
	
	return true;
}

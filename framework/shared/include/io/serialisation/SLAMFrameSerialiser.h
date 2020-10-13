/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_SLAMFRAMESERIALISER_H
#define IO_SLAMFRAMESERIALISER_H

#include "io/serialisation/Serialiser.h"

#include <cstdio>

namespace slambench {
	namespace io {
		
		class SLAMFrame;
		
		class SLAMFrameSerialiser : public Serialiser {
		public:
			SLAMFrameSerialiser(FILE *target);
			
			bool Serialise(SLAMFrame &frame);
		};
		
	}
}

#endif

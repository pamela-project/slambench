/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_SLAMFILESERIALISER_H
#define IO_SLAMFILESERIALISER_H

#include "io/serialisation/Serialiser.h"

#include <cstdio>

namespace slambench {
	namespace io {
		class SLAMFile;
		
		class SLAMFileSerialiser : public Serialiser {
		public:
			SLAMFileSerialiser(FILE *target);
		
			bool Serialise(SLAMFile &file);
		
			typedef void (*frame_callback_t)(int frame_num, int frame_count);
			frame_callback_t FrameCallback;
		
		private:
			bool SerialiseHeader(const SLAMFile &file);
			
			bool SerialiseFrames(SLAMFile &file);
		};
	}
}

#endif

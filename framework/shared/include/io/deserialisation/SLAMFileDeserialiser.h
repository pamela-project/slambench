/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_SLAMFILEDESERIALISER_H
#define IO_SLAMFILEDESERIALISER_H

#include "io/deserialisation/Deserialiser.h"

namespace slambench {
	namespace io {
		class FrameBuffer;
		class FrameBufferSource;
		class SLAMFile;
		class SLAMFrame;
		
		class SLAMFileDeserialiser : public Deserialiser {
		public:
			SLAMFileDeserialiser(FILE *file, FrameBufferSource *framebuffer_source);
			bool Deserialise(SLAMFile &target) const;
			
		private:
			bool DeserialiseHeader(SLAMFile &target) const;
			bool DeserialiseFrames(SLAMFile &target) const;
			bool DeserialiseFrame(SLAMFile &file, SLAMFrame *&frame) const;
			FrameBuffer *GetNextFramebuffer() const;
			
			FrameBufferSource *fb_source_;
		};
	}
}

#endif

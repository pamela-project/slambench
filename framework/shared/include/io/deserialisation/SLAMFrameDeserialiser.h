/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_SLAMFRAMEDESERIALISER_H
#define IO_SLAMFRAMEDESERIALISER_H

#include "io/deserialisation/Deserialiser.h"
#include "io/FrameSource.h"

namespace slambench {
	namespace io {
		class FrameBufferSource;
		class SensorCollection;
		
		class SLAMFrameDeserialiser : public Deserialiser, public slambench::io::FrameStream {
		public:
			SLAMFrameDeserialiser(FILE *file, SensorCollection &_sensors, FrameBufferSource *fb_source);
			SLAMFrame* GetNextFrame() override;
			bool HasNextFrame() override;

		private:
			SensorCollection &sensors_;
			FrameBufferSource *fb_source_;
		};
	}
}

#endif /* SLAMFRAMEDESERIALISER_H */


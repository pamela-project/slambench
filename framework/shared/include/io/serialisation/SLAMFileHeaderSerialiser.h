/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_SLAMFILEHEADERSERIALISER_H
#define IO_SLAMFILEHEADERSERIALISER_H

#include "io/serialisation/Serialiser.h"

namespace slambench {
	namespace io {
		class SLAMFile;
		class Sensor;
		
		class SLAMFileHeaderSerialiser : public Serialiser {
		public:
			SLAMFileHeaderSerialiser(FILE *target);
		
			bool Serialise(const SLAMFile &file);
		
		private:
			bool SerialiseHeader(const SLAMFile &file);
			bool SerialiseSensor(const Sensor &sensor);
		};
	}
}

#endif

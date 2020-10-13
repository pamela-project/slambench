/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef IO_SENSORCOLLECTIONDESERIALISER_H
#define IO_SENSORCOLLECTIONDESERIALISER_H

#include "io/deserialisation/Deserialiser.h"

namespace slambench {
	namespace io {
		class Sensor;
		class SensorCollection;
		
		class SensorCollectionDeserialiser : public Deserialiser {
		public:
			SensorCollectionDeserialiser(std::FILE *_file);
			bool Deserialise(SensorCollection &target) const;
			bool DeserialiseSensor(Sensor *&sensor) const;
		};
	}
}

#endif /* SENSORCOLLECTIONDESERIALISER_H */


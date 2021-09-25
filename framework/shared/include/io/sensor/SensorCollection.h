/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_SENSOR_SENSORCOLLECTION_H
#define IO_SENSOR_SENSORCOLLECTION_H

#include "io/sensor/Sensor.h"

#include <vector>
#include <cstring>

namespace slambench {
	namespace io {
		
		class Sensor;
		
		class SensorCollection {
		protected:
			typedef std::vector<Sensor*> container_t;

		public:
			container_t::iterator begin() { return container_.begin(); }
			container_t::iterator end() { return container_.end(); }
			
			container_t::const_iterator begin() const { return container_.begin(); }
			container_t::const_iterator end() const { return container_.end(); }
			
			Sensor &at(unsigned int sensor_idx);
			size_t size() const;
			
			Sensor *GetSensor(const Sensor::sensor_type_t &type);
			const Sensor *GetSensor(const Sensor::sensor_type_t &type) const;
			
			std::vector<Sensor*> GetSensors(const Sensor::sensor_type_t & type);
			
			void AddSensor(Sensor *sensor);
			
		private:
			container_t container_;
		
		};
		
	}
}

#endif

/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_SENSORDATABASE_H
#define IO_SENSORDATABASE_H

#include "io/sensor/Sensor.h"

#include <map>

namespace slambench {
	namespace io {
		
		class SensorDatabaseEntry {
		public:
			SensorDatabaseEntry(SensorSerialiser *s, SensorDeserialiser *ds, bool ground_truth = false, bool variable_size = false);
			
			SensorSerialiser *GetSerialiser() const { return serialiser_; }
			SensorDeserialiser *GetDeserialiser() const {return deserialiser_; }
			bool IsGroundTruth() const { return is_ground_truth_; }
			bool IsVariableSize() const { return is_variable_size_; }

		private:
			SensorSerialiser *serialiser_;
			SensorDeserialiser *deserialiser_;
			bool is_ground_truth_;
			bool is_variable_size_;
		};
		
		class SensorDatabase {
		public:
			SensorDatabaseEntry &Get(const Sensor::sensor_type_t &sensor_name);
			
			void RegisterSensor(const Sensor::sensor_type_t &sensor_name, const SensorDatabaseEntry &entry);
			static SensorDatabase *Singleton(void);
			
		private:
			std::map<Sensor::sensor_type_t, SensorDatabaseEntry> registrations_;
			static SensorDatabase *singleton_;
		};
		
		class SensorDatabaseRegistration {
		public:
			SensorDatabaseRegistration(const Sensor::sensor_type_t &name, const SensorDatabaseEntry &entry);
		};
	}
}

#endif /* IO_SENSORDATABASE_H */


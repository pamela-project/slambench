/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_SENSOR_SENSOR_H
#define IO_SENSOR_SENSOR_H

#include <string>
#include <Eigen/Eigen>
#include <ParameterComponent.h>

namespace slambench {
	namespace io {
		class Serialiser;
		class Deserialiser;
		class SLAMFrame;
		
		class Sensor : public ParameterComponent {
		public:
			typedef std::string sensor_name_t;
			typedef std::string sensor_type_t;
			typedef Eigen::Matrix4f pose_t;
			typedef uint8_t sensor_base_t;
			
			Sensor(const sensor_name_t &sensor_name, const sensor_type_t &sensor_type);
			virtual ~Sensor() = default;
		
			std::string Description;
			uint8_t Index;
			pose_t Pose;
			float Rate;
			float Delay; // Delay between sensor and baseline

			virtual size_t GetFrameSize(const SLAMFrame *frame) const = 0;
			const sensor_type_t &GetType() const;
			const sensor_name_t &GetName() const;
			
			void CopyPose(const Sensor *other);
			void CopyPose(const pose_t &other);
			
			bool IsVariableSize() const;
			bool IsGroundTruth() const;
			
		private:
			const sensor_name_t sensor_name_;
			const sensor_type_t sensor_type_;
		};
		
		class SensorSerialiser {
		public:
			virtual ~SensorSerialiser() = default;
			bool Serialise(Serialiser *serialiser, const Sensor *sensor);
			virtual bool SerialiseSensorSpecific(Serialiser *serialiser, const Sensor *sensor) = 0;
		};
		
		class SensorDeserialiser {
		public:
			virtual ~SensorDeserialiser() = default;
			bool Deserialise(const Sensor::sensor_name_t &sensor_name, const Sensor::sensor_type_t &type, const Deserialiser *d, Sensor **s);
			
			virtual bool InstantiateSensor(const Sensor::sensor_name_t &sensor_name, const Sensor::sensor_type_t & type, Sensor **s) = 0;
			virtual bool DeserialiseSensorSpecific(const Deserialiser *d, Sensor *s) = 0;
		};
		
	}
}

#endif

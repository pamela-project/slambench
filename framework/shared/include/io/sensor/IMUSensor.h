/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_IMUSENSOR_H
#define IO_IMUSENSOR_H

#include "io/sensor/Sensor.h"

namespace slambench {
	namespace io {
		class IMUSensor : public Sensor {
		public:
			static const sensor_type_t kIMUType;
			
			float GyroscopeNoiseDensity;
			float GyroscopeDriftNoiseDensity;
			float GyroscopeBiasDiffusion;

			float AcceleratorNoiseDensity;
			float AcceleratorBiasDiffusion;

			float AcceleratorSaturation;
			float GyroscopeSaturation;
			float AcceleratorDriftNoiseDensity;

			IMUSensor(const Sensor::sensor_name_t &sensor_name);
			size_t GetFrameSize(const SLAMFrame *frame) const override;

		};
	}
}

#endif /* IO_IMUSENSOR_H */


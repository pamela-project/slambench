/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_ACCELEROMETERSENSOR_H
#define IO_ACCELEROMETERSENSOR_H

#include "io/sensor/Sensor.h"

namespace slambench {
	namespace io {
		class AccelerometerSensor : public Sensor {
		public:
			const static sensor_type_t kAccType;
			
			AccelerometerSensor(const sensor_name_t &sensor_name);
			size_t GetFrameSize(const SLAMFrame *frame) const override;

		};
	}
}

#endif /* IO_ACCELEROMETERSENSOR_H */


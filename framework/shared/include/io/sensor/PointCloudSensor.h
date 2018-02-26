/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_POINTCLOUDSENSOR_H
#define IO_POINTCLOUDSENSOR_H

#include "io/sensor/Sensor.h"

namespace slambench {
	namespace io {
		class PointCloudSensor : public Sensor {
		public:
			static const sensor_type_t kPointCloudType;
			
			PointCloudSensor(const sensor_name_t &sensor_name);
			size_t GetFrameSize(const SLAMFrame *frame) const override;
		};
	}
}

#endif /* IO_POINTCLOUDSENSOR_H */


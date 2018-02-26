/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_GROUNDTRUTHSENSOR_H
#define IO_GROUNDTRUTHSENSOR_H

#include "io/sensor/Sensor.h"

namespace slambench {
	namespace io {
		class GroundTruthSensor : public Sensor {
		public:
			const static sensor_type_t kGroundTruthTrajectoryType;
			
			GroundTruthSensor(const Sensor::sensor_name_t &sensor_name);
			typedef Eigen::Matrix4f pose_t;
			size_t GetFrameSize(const SLAMFrame *frame) const override;

		};
	}
}

#endif /* IO_GROUNDTRUTHSENSOR_H */


/*

 Copyright (c) 2019 Intel Corp.

 This code is licensed under the MIT License.

 */

#ifndef IO_GYROSENSOR_H
#define IO_GYROSENSOR_H

#include "io/sensor/Sensor.h"

namespace slambench {
	namespace io {
		class GyroSensor : public Sensor {
		public:
			typedef float gyro_intrinsic_t[12];
			typedef float gyro_noise_variances_t[3];
			typedef float gyro_bias_variances_t[3];
			const static sensor_type_t kGyroType;

			gyro_intrinsic_t Intrinsic;
			gyro_noise_variances_t NoiseVariances;
			gyro_bias_variances_t BiasVariances;

			GyroSensor(const sensor_name_t &sensor_name);
			size_t GetFrameSize(const SLAMFrame *frame) const override;

		};
	}
}

#endif /* IO_GYROSENSOR_H */
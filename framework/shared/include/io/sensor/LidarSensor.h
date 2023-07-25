/*

 Copyright (c) 2023 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_LIDARSENSOR_H
#define IO_LIDARSENSOR_H

#include "io/sensor/Sensor.h"

namespace slambench {
	namespace io {
		class LidarSensor : public Sensor {
		public:
			static const sensor_type_t kLidarType;
			
			// More infos: https://felix.rohrba.ch/en/2015/four-essential-lidar-parameters/
			typedef float point_density_t;
			typedef float sidelap_t;
			typedef float altitude_above_ground_t;
			typedef float fov_t[2];
			typedef float angular_resolution_t;
			typedef int beam_num_t;

			point_density_t PointDensity;
			sidelap_t SideLap;
			altitude_above_ground_t AltitudeAboveGround;
			fov_t HorizontalFoV;
			fov_t VerticalFoV;
			angular_resolution_t HorizontalAngResolution;
			angular_resolution_t VerticalAngResolution;
			beam_num_t BeamNum;


			LidarSensor(const Sensor::sensor_name_t &sensor_name);
			size_t GetFrameSize(const SLAMFrame *frame) const override;
			void CopyHorizontalFoV(const fov_t &other);
			void CopyVerticalFoV(const fov_t &other);

		};
	}
}

#endif /* IO_LIDARSENSOR_H */
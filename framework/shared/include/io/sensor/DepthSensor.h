/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_DEPTH_SENSOR_H
#define IO_DEPTH_SENSOR_H

#include "io/sensor/CameraSensor.h"

namespace slambench {
	namespace io {
		class DepthSensor : public CameraSensor {
		public:
			DepthSensor(const Sensor::sensor_name_t &sensor_name);
			static const sensor_type_t kDepthType;
			
			typedef float disparity_params_t[2];
			typedef enum
			{
			    affine_disparity   = 0x00000000,
			    kinect_disparity   = 0X00000001,
			} disparity_type_t;
			
			disparity_type_t   DisparityType;
			disparity_params_t DisparityParams;
			
			void CopyDisparityParams(const DepthSensor *other);
			void CopyDisparityParams(const disparity_params_t &other);
		};
	}
}

#endif

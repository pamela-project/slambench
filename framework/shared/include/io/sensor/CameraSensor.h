/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_CAMERA_SENSOR_H
#define IO_CAMERA_SENSOR_H

#include "io/sensor/Sensor.h"
#include "io/FrameFormat.h"
#include "io/PixelFormat.h"


namespace slambench {
	namespace io {
		class CameraSensor : public Sensor {
		public:


			// Intrisics as "Focal Length" and "Principal Point Offset" : fx, fy, cx, cy
			typedef float intrinsics_t[4];


			// Distortion parameters :  radial and tangential factors = k1, k2, p1, p2, p3
			// More infos: https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
			typedef enum {NoDistortion, RadialTangential, Equidistant, KannalaBrandt} distortion_type_t;
			typedef float distortion_coefficients_t[5];

			const static sensor_type_t kCameraType;
			CameraSensor(const sensor_name_t &name, const sensor_type_t &type = kCameraType);
			
			// Resolution
			uint32_t Width;
			uint32_t Height;
			
			// Color setting
			frameformat::EFrameFormat FrameFormat;
			pixelformat::EPixelFormat PixelFormat;
			


			intrinsics_t        Intrinsics;
			distortion_type_t   DistortionType;
			distortion_coefficients_t RadialTangentialDistortion;
			distortion_coefficients_t EquidistantDistortion;
			distortion_coefficients_t Distortion;


			size_t GetFrameSize(const SLAMFrame *frame) const override;
			void CopyIntrinsics(const CameraSensor *other);
			void CopyIntrinsics(const intrinsics_t &other);
			void CopyRadialTangentialDistortion(const distortion_coefficients_t &other);
            void CopyEquidistantDistortion(const distortion_coefficients_t &other);
            void CopyDistortion(const distortion_coefficients_t &other, const distortion_type_t& type);
		};
	}
}


#endif

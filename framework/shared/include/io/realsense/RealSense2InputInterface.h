/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */
#ifdef DO_REALSENSE
#ifndef IO_REALSENSE2INPUTINTERFACE_H
#define IO_REALSENSE2INPUTINTERFACE_H

#include <io/InputInterface.h>
#include <io/sensor/SensorCollection.h>
#include "RealSense2FrameStream.h"

#include <string>
#include <librealsense2/hpp/rs_device.hpp>
#include <librealsense2/rs.hpp>

namespace slambench {
	namespace io {
	class CameraSensor;
	class DepthSensor;
		namespace realsense {
			class RealSense2FrameStream;
			
			class RealSense2InputInterface : public InputInterface {
			public:
				RealSense2InputInterface();
				FrameStream& GetFrames() override;
				SensorCollection& GetSensors() override;
                inline RealSense2FrameStream& GetStream() {return *stream_;}

			private:
                void BuildRGBSensor(const rs2::stream_profile& color_profile);
				void BuildDepthSensor(const rs2::stream_profile& depth_profile);
				void BuildAccelerometerGyroSensor(const rs2::stream_profile& motion_profile);
				void BuildStream();

				RealSense2FrameStream* stream_;
				SensorCollection sensors_;
				bool sensors_ready_;
                rs2::pipeline pipe_;
                //rs2::frameset frameset_;
            };
		}
	}
}

#endif /* IO_REALSENSE2INPUTINTERFACE_H */
#endif /* DO_REALSENSE */

/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_REALSENSE2FRAMESTREAM_H
#define IO_REALSENSE2FRAMESTREAM_H

#include "../FrameSource.h"

#include <map>
#include <vector>
#include <librealsense2/hpp/rs_device.hpp>
#include <librealsense2/hpp/rs_pipeline.hpp>

namespace slambench {
	namespace io {
		class Sensor;
		class CameraSensor;
		namespace realsense {
			
			/*
			 This class represents a stream of frames from an OpenNI2 device.
			 Once the stream has been created, each desired sensor should be
			 activated.
			 */
			
			class RealSense2FrameStream : public FrameStream {
			public:
				RealSense2FrameStream(rs2::pipeline &pipe);
				~RealSense2FrameStream();
                bool ActivateSensor(rs2_stream stream, Sensor* sensor);
				SLAMFrame* GetNextFrame() override;
				bool HasNextFrame() override;
            //
			private:
                rs2::pipeline &pipe_;
                rs2::frameset frameset_;
                std::vector<rs2::frame> new_frames_;
				std::map<rs2_stream, Sensor*> sensor_map_;
			};
			
		}
	}
}

#endif /* IO_REALSENSE2FRAMESTREAM_H */


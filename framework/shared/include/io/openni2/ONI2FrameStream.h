/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_ONI2FRAMESTREAM_H
#define IO_ONI2FRAMESTREAM_H

#include "../FrameSource.h"

#include <map>
#include <vector>

namespace openni {
	class Device;
	class VideoStream;
}

namespace slambench {
	namespace io {
		class Sensor;
		class CameraSensor;
		namespace openni2 {
			
			/*
			 This class represents a stream of frames from an OpenNI2 device.
			 Once the stream has been created, each desired sensor should be
			 activated.
			 */
			
			class ONI2FrameStream : public FrameStream {
			public:
				ONI2FrameStream(openni::Device *device);
				
				bool ActivateSensor(CameraSensor *sensor);
				bool StartStreams();
				
				SLAMFrame* GetNextFrame() override;
				bool HasNextFrame() override;

			private:
				openni::Device *device_;
				
				std::vector<openni::VideoStream*> streams_;
				std::map<openni::VideoStream*, Sensor*> sensor_map_;
			};
			
		}
	}
}

#endif /* IO_ONI2FRAMESTREAM_H */


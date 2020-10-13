/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_ONI15FRAMESTREAM_H
#define IO_ONI15FRAMESTREAM_H

#include "../FrameSource.h"

#include <map>
#include <vector>

namespace xn {
	class DepthGenerator;
	class ImageGenerator;
	class MapGenerator;
	class Context;
	class DepthMetaData;
	class ImageMetaData;
}

namespace slambench {
	namespace io {
		class Sensor;
		class CameraSensor;
		namespace openni15 {
			
			/*
			 This class represents a stream of frames from OpenNI15 generators.
			 Once the stream has been created, each desired sensor should be
			 activated.
			 */
			class ONI15FrameStream : public FrameStream {
			public:
				ONI15FrameStream(xn::Context *context) ;
				
				bool ActivateSensor(CameraSensor *sensor);
				bool StartStreams();
				
				SLAMFrame* GetNextFrame() override;
				bool HasNextFrame() override;

			private:
				xn::Context * context_;
				xn::DepthGenerator* depth_generator_;
				xn::ImageGenerator* image_generator_;
				xn::DepthMetaData* DMD_ ;
				xn::ImageMetaData* IMD_ ;
				std::map<xn::MapGenerator*, Sensor*> _sensor_map;
			};
		}
	}
}

#endif /* IO_ONI2FRAMESTREAM_H */


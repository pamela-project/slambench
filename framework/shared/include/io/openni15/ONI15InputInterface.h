/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_ONI15INPUTINTERFACE_H
#define IO_ONI15INPUTINTERFACE_H

#include "../InputInterface.h"
#include "../sensor/SensorCollection.h"

#include <string>


namespace xn {
	class Context;
}
namespace slambench {
	namespace io {
	class CameraSensor;
	class DepthSensor;
		namespace openni15 {
			class ONI15FrameStream;
			
			class ONI15InputInterface : public InputInterface {
			public:
				ONI15InputInterface();
				
				FrameStream& GetFrames() override;
				SensorCollection& GetSensors() override;

			private:
				void BuildSensors();
				CameraSensor *BuildCameraSensor() ;
				DepthSensor *BuildDepthSensor() ;

				void BuildStream();

				xn::Context * _context;
				ONI15FrameStream *_stream;
				
				SensorCollection _sensors;
				bool _sensors_ready;
			};
		}
	}
}

#endif /* IO_ONI2INPUTINTERFACE_H */


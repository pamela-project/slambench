/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */
#ifdef DO_OPENNI15
#ifndef IO_ONI15INPUTINTERFACE_H
#define IO_ONI15INPUTINTERFACE_H

#include <io/InputInterface.h>
#include <io/sensor/SensorCollection.h>

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

				xn::Context * context_;
				ONI15FrameStream *stream_;
				
				SensorCollection sensors_;
				bool sensors_ready_;
			};
		}
	}
}

#endif /* IO_ONI15INPUTINTERFACE_H */
#endif /* DO_OPENNI15 */


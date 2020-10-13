/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */
#ifdef DO_OPENNI20
#ifndef IO_ONI2INPUTINTERFACE_H
#define IO_ONI2INPUTINTERFACE_H

#include <io/InputInterface.h>
#include <io/sensor/SensorCollection.h>

#include <string>

namespace openni {
	class Device;
	class SensorInfo;
}

namespace slambench {
	namespace io {
	class CameraSensor;
	class DepthSensor;
		namespace openni2 {
			class ONI2FrameStream;
			
			class ONI2InputInterface : public InputInterface {
			public:
				ONI2InputInterface();
				ONI2InputInterface(const std::string& oni2_filename);
				
				FrameStream& GetFrames() override;
				SensorCollection& GetSensors() override;

			private:
				void BuildSensors();
				void BuildStream();
				CameraSensor *BuildCameraSensor(const openni::SensorInfo *sensor_info) ;
				DepthSensor *BuildDepthSensor(const openni::SensorInfo *sensor_info) ;


				openni::Device *device_;
				ONI2FrameStream *stream_;
				
				SensorCollection sensors_;
				bool sensors_ready_;
			};
		}
	}
}

#endif /* IO_ONI2INPUTINTERFACE_H */
#endif /* DO_OPENNI20 */
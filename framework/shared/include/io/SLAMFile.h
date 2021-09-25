/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef IO_SLAMFILE_H
#define IO_SLAMFILE_H

#include "FrameSource.h"
#include "sensor/SensorCollection.h"
#include "serialisation/SLAMFileSerialiser.h"

#include <string>
#include <vector>

namespace slambench {
	namespace io {
		class FrameBufferSource;
		class SLAMFrame;
		class Sensor;
		
		class SLAMFile : public FrameCollection
		{
		public:
			typedef SensorCollection sensor_container_t;
			typedef std::vector<SLAMFrame*> frame_container_t;
			
			typedef char magic_num_t[5];
			typedef uint32_t version_t;
			
			sensor_container_t Sensors;
			
			static const int Version = 1;
			static const magic_num_t MagicNum;
			
			Sensor *GetSensor(const Sensor::sensor_type_t &type);
			
			SLAMFrame* GetFrame(unsigned int index) override;
			unsigned int GetFrameCount() override;
			
			void AddFrame(SLAMFrame *frame);

			static SLAMFile *Read(const std::string &filename, FrameBufferSource &fb_source);
			static bool Write(const std::string &filename, SLAMFile &file, SLAMFileSerialiser::frame_callback_t callback = nullptr);
			
		private:
			void AddGroundTruthFrame(SLAMFrame *frame);
			void AddRegularFrame(SLAMFrame *frame);
			
			frame_container_t frames_;
		};
	}
}

#endif

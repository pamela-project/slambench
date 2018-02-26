/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef IO_INPUTINTERFACE_H
#define IO_INPUTINTERFACE_H

#include <cstring>

#include <io/FrameSource.h>
#include <io/SLAMFile.h>

#include "sensor/SensorCollection.h"
#include "deserialisation/SLAMFrameDeserialiser.h"

namespace slambench {
	namespace io {
		
		class InputInterface {
		public:
			virtual ~InputInterface();
			
			virtual SensorCollection &GetSensors() = 0;
			virtual FrameStream &GetFrames() = 0;
		};
		
		class BasicInputInterface : public InputInterface {
		public:
			BasicInputInterface(FrameStream &frames, SensorCollection &sensors);
			
			FrameStream& GetFrames() override;
			SensorCollection& GetSensors() override;
		private:
			FrameStream &_frames;
			SensorCollection &_sensors;
		};
		
		class FileInputInterface : public InputInterface {
		public:
			FileInputInterface(SLAMFile &input_file);
			
			FrameStream& GetFrames() override;
			SensorCollection& GetSensors() override;
		private:
			SLAMFile &_file;
			FrameCollectionStream _stream;
		};
		
		class FileStreamInputInterface : public InputInterface {
		public:
			FileStreamInputInterface(FILE* input_file, FrameBufferSource *fb_source);
			
			FrameStream &GetFrames() override;
			SensorCollection &GetSensors() override;
		private:
			SLAMFrameDeserialiser _deserialiser;
			SensorCollection _sensors;
		};
	}
}

#endif

/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef IO_SLAMFRAME_H
#define IO_SLAMFRAME_H

#include <string>

#include <cstdint>
#include <cstdio>
#include <cstddef>

#include "TimeStamp.h"
#include "PixelFormat.h"

namespace slambench {
	namespace io {
		class Sensor;
		class FrameBuffer;
		
		class SLAMFrame {
		public:
			virtual ~SLAMFrame();
		
			TimeStamp Timestamp;
			Sensor *FrameSensor;
			
			size_t GetSize() const;
			
			void SetVariableSize(uint32_t size);
			uint32_t GetVariableSize() const;
			
			virtual void *GetData() = 0;
			virtual void FreeData() = 0;
			
		private:
			uint32_t size_if_variable_sized_;
		};
		
		class SLAMInMemoryFrame : public SLAMFrame {
		public:
			void *Data;
		
			void *GetData() override;
			void FreeData() override;
		};
		
		class SLAMFileFrame : public SLAMFrame {
		public:
			SLAMFileFrame();
			
			typedef void (*callback_t)(SLAMFileFrame *, void *);
			callback_t ProcessCallback;
			
			std::string Filename;
			
			void *GetData() override;
			void FreeData() override;
			
		protected:
			virtual void *LoadFile() = 0;
		private:
			void *_data;
		};
		
		class TxtFileFrame : public SLAMFileFrame {
		public:
			
			pixelformat::EPixelFormat InputPixelFormat;
			
			
		protected:
			void *LoadFile() override;
			
		private:
			void *LoadCameraFile();
		};
		
		class ImageFileFrame : public SLAMFileFrame {
		protected:
			void *LoadFile() override;
			
		private:
			void *LoadPng();
			void *LoadPbm();
		};
		
		class DeserialisedFrame : public SLAMFrame {
		public:
			DeserialisedFrame(FrameBuffer &buffer, FILE *file);
			
			void SetOffset(size_t data_offset);
			
			void *GetData() override;
			void FreeData() override;
			FrameBuffer& getFrameBuffer() {
				return _buffer;
			}
			
		private:
			FrameBuffer &_buffer;
			FILE *_file;
			size_t _offset;
		};
		
	}
}

#endif

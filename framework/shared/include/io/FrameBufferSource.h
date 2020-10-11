/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef IO_FRAMEBUFFERSOURCE_H
#define IO_FRAMEBUFFERSOURCE_H

#include <io/FrameBuffer.h>

namespace slambench {
	namespace io {
		class FrameBufferSource {
		public:
			virtual FrameBuffer *Next() = 0;
			virtual ~FrameBufferSource() {};
		};
		
		class SingleFrameBufferSource : public FrameBufferSource {
		public:
			FrameBuffer *Next() override;
			
		private:
			FrameBuffer fb_;
		};
	}
}

#endif

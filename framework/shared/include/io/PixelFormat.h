/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef IO_PIXELFORMAT_H
#define IO_PIXELFORMAT_H

#include <cstddef>
#include <string>

namespace slambench {
	namespace io {
		namespace pixelformat {
			enum EPixelFormat {
				UNKNOWN,
				G_I_8,
				RGB_III_888,
				D_I_8,
				D_F_32,
				
				D_I_16,
				D_F_64,
				
				RGBA_IIII_8888
			};
			
			EPixelFormat Parse(const std::string &fmt);
			std::string TypeOf(EPixelFormat);
			size_t GetPixelSize(EPixelFormat);
			bool IsRGB(EPixelFormat);
			bool IsGrey(EPixelFormat);
			bool ISDepth(EPixelFormat);
		}
	}
}

#endif

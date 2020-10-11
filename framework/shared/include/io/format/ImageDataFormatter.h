/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef IO_IMAGEDATAFORMATTER_H
#define IO_IMAGEDATAFORMATTER_H

#include "io/sensor/CameraSensor.h"
#include "io/sensor/DepthSensor.h"
#include "io/format/DataFormatter.h"

#include <cassert>

namespace slambench {
	namespace io {
		template<pixelformat::EPixelFormat pxl_format> class PixelData {
			PixelData(void *data);
		};
		
		template<> class PixelData<pixelformat::RGB_III_888> {
		public:
			PixelData(const void *data) { 
				const char *cdata = (const char*)data;
                r_ = (uint8_t) cdata[0];
                g_ = (uint8_t) cdata[1];
                b_ = (uint8_t) cdata[2];
			}
			
			static const size_t Size = 3;
					
		private:
			uint8_t r_, g_, b_;
			
		public:
			static_assert(sizeof(r_) + sizeof(g_) + sizeof(b_) == Size, "Size mismatch!");
			
			uint8_t R() const { return r_; }
			uint8_t G() const { return g_; }
			uint8_t B() const { return b_; }
				
		};
		
		template<> class PixelData<pixelformat::D_F_32> {
		public:
			PixelData(const void* data) {
				const float *fdata = (const float*)data;
				_d = *fdata;
			}
			
		private:
			float _d;
			
		public:
			float D() const { return _d; }
			
			static const size_t Size = 4;
			static_assert(sizeof(_d) == Size, "Size mismatch");
		};
				
		template<pixelformat::EPixelFormat pxl_format> class RasterImageFormatter : public DataFormatter {
		public:
			typedef PixelData<pxl_format> pixel_t;
			
			RasterImageFormatter(Sensor *sensor, void *data) : DataFormatter(sensor, data) {
				assert(sensor->GetType() == CameraSensor::kCameraType || sensor->GetType() == DepthSensor::kDepthType);
			}
			
			CameraSensor *GetCamera() { return (CameraSensor*)DataFormatter::GetSensor(); }
			size_t Line() const { return GetCamera()->Width * pixel_t::Size; }
			size_t Stride() const { return pixel_t::Size; }
			
			pixel_t Get(uint32_t x, uint32_t y) const {
				const char *data = (const char*)Data();
				data += Line() * y;
				data += Stride() * x;
				
				return pixel_t(data);
			}
		};		
	}
}

#endif /* IO_IMAGEDATAFORMATTER_H */


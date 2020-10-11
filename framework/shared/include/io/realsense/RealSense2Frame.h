/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_REALSENSE2FRAME_H
#define IO_REALSENSE2FRAME_H

#include "../SLAMFrame.h"
#include <librealsense2/hpp/rs_frame.hpp>


namespace slambench {
	namespace io {
		class Sensor;
		namespace realsense {
			class RealSense2Frame : public SLAMFrame {
			public:
				RealSense2Frame(Sensor *sensor, const rs2::frame& frameref);
				void* GetData() override;
				void FreeData() override;
			};
		}
	}
}

#endif /* IO_REALSENSE2FRAME_H */


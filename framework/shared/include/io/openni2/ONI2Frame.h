/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_ONIFRAME_H
#define IO_ONIFRAME_H

#include "../SLAMFrame.h"

#include <OpenNI.h>

namespace slambench {
	namespace io {
		class Sensor;
		namespace openni2 {
			class ONI2Frame : public SLAMFrame {
			public:
				ONI2Frame(Sensor *sensor, const openni::VideoFrameRef &frameref);
				void* GetData() override;
				void FreeData() override;
			};
		}
	}
}

#endif /* IO_ONIFRAME_H */


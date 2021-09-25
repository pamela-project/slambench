/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_ONIFRAME15_H
#define IO_ONIFRAME15_H

#include "../SLAMFrame.h"

#define linux true

#include <XnOpenNI.h>
#include <XnLog.h>
#include <XnCppWrapper.h>

namespace slambench {
	namespace io {
		class Sensor;
		namespace openni15 {
			class ONI15Frame : public SLAMFrame {
			public:
				ONI15Frame(Sensor *sensor, const xn::OutputMetaData *frameref);
				~ONI15Frame();
				void* GetData() override;
				void FreeData() override;
			};
		}
	}
}

#endif /* IO_ONIFRAME15_H */


/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_EVENTCAMERASENSOR_H
#define IO_EVENTCAMERASENSOR_H

#include "io/sensor/Sensor.h"

namespace slambench {
	namespace io {
		class EventCameraSensor : public Sensor {
		public:
			EventCameraSensor(const sensor_name_t &sensor_name);
			static const sensor_type_t kEventCameraType;
			
			int Width;
			int Height;
			
			size_t GetCoordinateSize() const;
			size_t GetFrameSize(const SLAMFrame *frame) const override;
		};
	}
}

#endif /* IO_EVENTCAMERASENSOR_H */


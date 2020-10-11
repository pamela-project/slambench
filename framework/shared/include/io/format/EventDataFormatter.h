/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_EVENTDATAFORMATTER_H
#define IO_EVENTDATAFORMATTER_H

#include "io/format/DataFormatter.h"
#include "io/sensor/EventCameraSensor.h"

#include <cstdint>

namespace slambench {
	namespace io {
		class EventData {
		public:
			uint32_t X, Y;
			bool Raised;
		};
		
		class EventDataFormatter : public DataFormatter {
		private:
			uint8_t coordsize_;
		public:
			EventDataFormatter(Sensor *sensor, void *data) : DataFormatter(sensor, data) { coordsize_ = ((EventCameraSensor*)sensor)->GetCoordinateSize(); }
			
			EventData Get() const {
				EventData data;
				const char *ptr = (const char*)Data();
				switch(coordsize_) {
					case 1: 
						data.X = *((uint8_t*)ptr++);
						data.Y = *((uint8_t*)ptr++);
						break;
					case 2:
						data.X = *((uint16_t*)ptr++);
						data.Y = *((uint16_t*)ptr++);
						break;
					case 4:
						data.X = *((uint32_t*)ptr++);
						data.Y = *((uint32_t*)ptr++);
						break;
				}
				data.Raised = *ptr;
				
				return data;
			}
			
			void Put(const EventData &data) {
				char *ptr = (char*)Data();
				switch(coordsize_) {
					case 1: 
						*((uint8_t*)ptr++) = data.X;
						*((uint8_t*)ptr++) = data.Y;
						break;
					case 2:
						*((uint16_t*)ptr++) = data.X;
						*((uint16_t*)ptr++) = data.Y;
						break;
					case 4:
						*((uint32_t*)ptr++) = data.X;
						*((uint32_t*)ptr++) = data.Y;
						break;
				}
				*ptr = data.Raised;
			}
		};
		
	}
}

#endif /* IO_EVENTDATAFORMATTER_H */


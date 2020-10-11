/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef IO_DATAFORMATTER_H
#define IO_DATAFORMATTER_H

namespace slambench {
	namespace io {
		class Sensor;
		
		class DataFormatter {
		public:
			DataFormatter(Sensor *sensor, void *data);
			
			Sensor *GetSensor() { return sensor_; }
			const Sensor *GetSensor() const { return sensor_; }

		protected:
			const void *Data() const { return data_; }
			void *Data() { return data_; }

		private:
			void *data_;
			Sensor *sensor_;
		};
	}
}

#endif /* IO_DATAFORMATTER_H */


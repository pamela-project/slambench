/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_CAMERASENSORFINDER_H
#define IO_CAMERASENSORFINDER_H

#include <functional>
#include <vector>
#include <map>
#include <string>

namespace slambench {
	namespace io {
		class SensorCollection;
		class CameraSensor;
		
		typedef std::pair<std::string, std::string> CameraSensorFilter;
		
		class CameraSensorFinder {
		public:
			typedef std::function<bool(const CameraSensor *sensor, const std::string &value)> filter_lambda_t;
			
			CameraSensorFinder();
			
			bool TestFilter(const CameraSensor *sensor, const CameraSensorFilter &filter);
			std::vector<CameraSensor*> Find(const SensorCollection& sensors, const std::initializer_list<CameraSensorFilter> &filters);
			CameraSensor *FindOne(const SensorCollection &sensors, const std::initializer_list<CameraSensorFilter> &filters);
			
		private:
			void PrepareEvaluators();
			
			std::map<std::string, filter_lambda_t> filter_evaluators_;
		};
	}
}

#endif /* IO_CAMERASENSORFINDER_H */


/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef PHASE_H
#define PHASE_H

#include "MetricValue.h"

#include <unordered_map>
#include <string>

namespace slambench {
	namespace metrics {
		class Metric;
		class MetricManager;
		
		// Class representing a phase (i.e., a duration) of an algorithm
		class Phase {
		public:
			Phase(MetricManager *mgr, const std::string &phasename);
			
			void Begin();
			void End();
			const std::string &GetName() { return name_; }
			
		private:
			const std::string name_;
			MetricManager *manager_;
		};
	}
}

#endif /* PHASE_H */


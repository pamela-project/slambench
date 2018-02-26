/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef METRIC_H
#define METRIC_H

#include "MetricValue.h"
#include "SLAMBenchLibraryHelper.h"
#include "io/SLAMFrame.h"
#include "values/Value.h"

#include <functional>
#include <string>
#include <unordered_map>

namespace slambench {
	namespace metrics {
		class Phase;
		using values::Value;
		
		// Class representing a metric such as power, time, memory usage, etc.
		class Metric {
		public:
			Metric(const std::string &name);
			virtual ~Metric();
			
			virtual void MeasureStart(Phase *phase) = 0;
			virtual void MeasureEnd(Phase *phase) = 0;
			
			virtual Value *GetValue(Phase *phase) = 0;
			virtual const values::ValueDescription &GetValueDescription() const = 0;
			
			const std::string &GetName() const { return name_; }
			virtual const std::string &GetDescription() const = 0;
		 	
		private:
			const std::string name_;
			
		};
	}
}

#endif /* METRIC_H */


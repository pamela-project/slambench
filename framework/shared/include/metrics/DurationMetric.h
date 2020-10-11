/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef DURATIONMETRIC_H
#define DURATIONMETRIC_H

#include "Metric.h"


#include <functional>
#include <string>
#include <unordered_map>

namespace slambench {
	namespace values {
		class Value;
	}
	namespace metrics {
		class Phase;
		
		class DurationMetric : public Metric {
		public:
			DurationMetric();
			
			virtual void MeasureStart(Phase* phase) override;
			virtual void MeasureEnd(Phase* phase) override;
			values::Value *GetValue(Phase *phase) override;
			const slambench::values::ValueDescription& GetValueDescription() const override;
			const std::string &GetDescription() const override;

		private:
			std::unordered_map<Phase *, time_t> phase_start_;
			std::unordered_map<Phase *, time_t> phase_end_;
			uint64_t getTime() const;
		};
		
	}
}

#endif /* DURATIONMETRIC_H */


/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef POWERMETRIC_H
#define POWERMETRIC_H

#include "metrics/Metric.h"
#include "metrics/power_utils/PowerMonitor.h"

namespace slambench {
	namespace metrics {
		class PowerMetric : public Metric {

		public:
			PowerMetric();

			const slambench::values::ValueDescription& GetValueDescription() const override;
			const std::string& GetDescription() const override;
			void MeasureStart(Phase* phase) override;
			void MeasureEnd(Phase* phase) override;
			slambench::values::Value *GetValue(Phase* phase) override;

        private :
            PowerMonitor *pm_ = nullptr;
            slambench::values::Value *last_res_;
		};
	}
}

#endif /* POWERMETRIC_H */


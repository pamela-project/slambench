/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef ATEMETRIC_H
#define ATEMETRIC_H

#include "metrics/Metric.h"

namespace slambench {
namespace outputs {
class TrajectoryInterface;
}
}

namespace slambench {
	namespace metrics {
		class ATEMetric : public Metric {
		public:
			ATEMetric(const slambench::outputs::TrajectoryInterface *tested_trajectory, const slambench::outputs::TrajectoryInterface *ground_truth);
			const slambench::values::ValueDescription& GetValueDescription() const override;
			const std::string& GetDescription() const override;
			void MeasureStart(Phase* phase) override;
			void MeasureEnd(Phase* phase) override;
			Value *GetValue(Phase* phase) override;

		private:
			const outputs::TrajectoryInterface *trajectory_;
			const outputs::TrajectoryInterface *ground_truth_;
			values::TrajectoryValue::pose_container_t latest_trajectory_;
			slambench::TimeStamp next_gt_ts_;
		};
	}
}

#endif /* ATEMETRIC_H */


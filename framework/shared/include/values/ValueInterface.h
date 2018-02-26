/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef VALUEINTERFACE_H
#define VALUEINTERFACE_H

#include "metrics/MetricManager.h"
#include "metrics/Metric.h"
#include "metrics/Phase.h"

namespace slambench {
	namespace values {
		class ValueInterface {
		public:
			virtual ~ValueInterface();
			virtual const values::Value *Get() = 0;
			virtual const values::ValueDescription &GetDescription() const = 0;
		};
		
		class MetricValueInterface : public ValueInterface {
		public:
			MetricValueInterface(metrics::MetricManager &manager, metrics::Metric &metric, metrics::Phase &phase) : manager_(manager), metric_(metric), phase_(phase) {}
			virtual ~MetricValueInterface() {}
			const values::Value *Get() override { return manager_.GetLastFrame()->GetPhaseData(&phase_).at(&metric_); }
			const values::ValueDescription &GetDescription() const  override { return metric_.GetValueDescription(); }

		private:
			metrics::MetricManager &manager_;
			metrics::Metric &metric_;
			metrics::Phase &phase_;
		};
		
		class OutputValueInterface : public ValueInterface {
		public:
			OutputValueInterface(outputs::BaseOutput &output) : output_(output) {}
			virtual ~OutputValueInterface() {}
			const values::Value* Get() override { return output_.GetMostRecentValue().second; }
			const values::ValueDescription &GetDescription() const override { return output_.GetValueDescription(); }

		private:
			outputs::BaseOutput &output_;
		};
	}
}

#endif /* VALUEINTERFACE_H */


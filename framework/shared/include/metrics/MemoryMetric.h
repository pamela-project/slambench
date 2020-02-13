/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef MEMORYMETRIC_H
#define MEMORYMETRIC_H

#include "Metric.h"
#include "memory_utils/CUDAMonitor.h"

#include <functional>
#include <string>
#include <unordered_map>


namespace slambench {
	namespace metrics {
		class Phase;

		class MemoryMetric : public Metric {
		public:
			MemoryMetric();
			~MemoryMetric();
			
			virtual void MeasureStart(Phase* phase) override;
			virtual void MeasureEnd(Phase* phase) override;
			
			values::Value *GetValue(Phase* phase) override;
			const values::ValueDescription &GetValueDescription() const override;
			const std::string &GetDescription() const override;
			
		//private:
			
			slambench::values::ValueDescription desc;
			CUDAMonitor cuda_monitor;

		private:
			std::unordered_map<Phase *, size_t> CPU_Usage_;
			std::unordered_map<Phase *, size_t> GPU_Usage_;

		};
		
	}
}

#endif /* CPUMEMORYMETRIC_H */


/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef METRICSMANAGER_H
#define METRICSMANAGER_H

#include "Phase.h"
#include "FrameData.h"

#include <map>
#include <set>
#include <string>
#include <vector>

namespace slambench {
	namespace metrics {
		
		class Metric;
		class Phase;
		
		class MetricManager {
		public:
			MetricManager();
			~MetricManager();
			
			typedef std::vector<Metric*> metric_list_t;
			typedef std::vector<Phase*> phase_list_t;
			
			void AddPhaseMetric(Metric *m);
			void AddFrameMetric(Metric *m);
			
			void AddPhase(Phase *p);
			void AddPhase(const std::string &phase_name);
			
			void BeginPhase(Phase *phase);
			void EndPhase(Phase *phase);
			
			Phase *GetPhase(const std::string &phasename);
			Phase *GetFramePhase() { return &frame_phase_; }
			Phase *GetInitPhase() { return &init_phase_; }
			const PhaseData *GetInitPhaseData() { return &init_data_; }
			phase_list_t GetPhases() { return phases_; }
			metric_list_t GetPhaseMetrics() { return phase_metrics_; }
			metric_list_t GetFrameMetrics() { return frame_metrics_; }
			
			FrameData *GetFrame(uint64_t frame_number);
			FrameData *GetLastFrame();
			
			std::string GetHeader() const;

			void BeginFrame();
			void EndFrame();

			void BeginInit();
			void EndInit();
		private:
			metric_list_t frame_metrics_;
			metric_list_t phase_metrics_;
			std::set<Metric*> all_metrics_;
			
			phase_list_t phases_;
			
			std::vector<FrameData*> frame_data_;
			PhaseData init_data_;
			Phase init_phase_;
			Phase frame_phase_;
			
			uint64_t frame_counter_;
		};
	}
}

#endif /* METRICSMANAGER_H */


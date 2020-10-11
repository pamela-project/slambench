/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEDATA_H
#define FRAMEDATA_H

#include <map>

namespace slambench {
	namespace values {
		class Value;
	}
	namespace metrics {
		class Metric;
		class Phase;
		
		typedef std::map<Metric *, values::Value*> PhaseData;
		
		class FrameData {
		public:
		    FrameData() = default;
		    ~FrameData() {phase_data_.clear();frame_data_.clear();};
			PhaseData &GetPhaseData(Phase *p) { return phase_data_[p]; }
			PhaseData &GetFrameData() { return frame_data_; }
			void Clear() { phase_data_.clear(); frame_data_.clear(); }
			
		private:
			std::map<Phase*, PhaseData> phase_data_;
			PhaseData frame_data_;
		};
	}
}

#endif /* FRAMEDATA_H */


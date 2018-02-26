/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef OUTPUTMANAGER_H
#define OUTPUTMANAGER_H

#include "utils.h"
#include "Output.h"

#include "io/SLAMFile.h"
#include "io/SLAMFrame.h"
#include "io/sensor/SensorCollection.h"

#include <map>
#include <mutex>
#include <string>

namespace slambench {
	namespace outputs {
		class Output;
		
		class OutputManager {
		public:
			typedef std::map<std::string, BaseOutput*> output_map_t;
			typedef output_map_t::iterator output_map_iterator_t;
			
			~OutputManager();
			

			void LoadGTOutputsFromSLAMFile(io::SLAMFile *file);
			void LoadGTOutputsFromSLAMFile(io::SensorCollection &sensors, io::FrameCollection *gt_frames, bool with_point_cloud);
			
			void RegisterOutput(BaseOutput *output);
			
			BaseOutput *GetOutput(const std::string &outputname);
			BaseOutput *GetMainOutput(slambench::values::ValueType);
			
			output_map_iterator_t begin() { return output_map_.begin(); }
			output_map_iterator_t end() { return output_map_.end(); }
			
			bool WriteFile(const std::string &filename);
			size_t OutputCount() const { return output_map_.size(); }
			
			FastLock &GetLock() { return lock_; }
		private:
			std::map<std::string, BaseOutput*> output_map_;
			FastLock lock_;
		};
	}
}

#endif /* OUTPUTMANAGER_H */

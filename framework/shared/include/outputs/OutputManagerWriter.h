/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef OUTPUTMANAGERWRITER_H
#define OUTPUTMANAGERWRITER_H

#include "io/SLAMFile.h"
#include "TimeStamp.h"

#include <map>
#include <string>

namespace slambench {
	namespace values {
		class Value;
		class PoseValue;
		class PointCloudValue;
	}
	namespace outputs {
		class BaseOutput;
		class OutputManager;
		
		class OutputManagerWriter {
		public:
			slambench::io::SLAMFile *GetFile(OutputManager &outman);
			bool Write(OutputManager &outman, const std::string &filename);
			
		private:
			bool CreateSensors(OutputManager &outman, slambench::io::SLAMFile &file);
			
			bool CreateFrames(OutputManager &outman, slambench::io::SLAMFile &file);
			slambench::io::SLAMFrame *CreateFrame(BaseOutput *output, TimeStamp ts,const values::Value* value);
			slambench::io::SLAMFrame *CreatePoseFrame(BaseOutput *output, TimeStamp ts,const values::PoseValue* value);
			slambench::io::SLAMFrame *CreatePointCloudFrame(BaseOutput *output, TimeStamp ts,const values::PointCloudValue* value);
			
			slambench::io::Sensor *CreateSensor(const BaseOutput *output);
			slambench::io::Sensor *CreatePoseSensor(const BaseOutput *output);
			slambench::io::Sensor *CreatePointCloudSensor(const BaseOutput *output);
			
			bool SerialiseFile(slambench::io::SLAMFile &file, const std::string &filename);
			
			std::map<BaseOutput*, slambench::io::Sensor*> output_map_;
		};
	}
}

#endif /* OUTPUTMANAGERWRITER_H */


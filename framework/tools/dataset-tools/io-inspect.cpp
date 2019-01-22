/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/SLAMFile.h"
#include "io/SLAMFrame.h"
#include "io/FrameBufferSource.h"
#include "io/deserialisation/SLAMFileDeserialiser.h"
#include "io/format/ImageDataFormatter.h"

#include <iostream>
#include <iomanip>

using namespace slambench::io;

slambench::io::SingleFrameBufferSource single_fb_source;


void printFrame (SLAMFrame *frame , unsigned int index ) {
	std::cout << "Frame " << index << std::endl;
	std::cout << " -> Timestamp " << std::fixed << std::setprecision(9) << (double)frame->Timestamp.S + (frame->Timestamp.Ns / 1000000000.0) << std::endl;

	if (frame->FrameSensor->GetType() == CameraSensor::kCameraType) {
		slambench::io::CameraSensor * rgb_sensor = dynamic_cast<slambench::io::CameraSensor *> (frame->FrameSensor) ;
	}
	std::cout << " -> Sensor " << frame->FrameSensor->GetType() << " " <<  (uint32_t)frame->FrameSensor->Index <<  "  (" <<  frame->FrameSensor->Description.c_str() <<  ")" << std::endl;


}

void Deserialise(const std::string &filename, SLAMFile &file) {
	FILE *input_file = fopen(filename.c_str(), "r");
	SLAMFileDeserialiser deserialiser(input_file, &single_fb_source);
	
	if(deserialiser.Deserialise(file)) {
		printf("Deserialisation success\n");
	} else {
		printf("Deserialisation failure\n");
	}
}

void Check(SLAMFile &file) {

	std::cout << "File version: " <<  SLAMFile::Version   << std::endl;
	std::cout << "Sensor count: " <<  file.Sensors.size() << std::endl;
	std::cout << "Frame count: " <<  file.GetFrameCount() << std::endl;
	
	for(const auto &sensor : file.Sensors) {
		unsigned int count = 0;
		for(unsigned int index = 0; index < file.GetFrameCount(); ++index) {
			SLAMFrame *frame = file.GetFrame(index);
			if (frame->FrameSensor->Index == sensor->Index) count++;
		}
		if (sensor->GetType() == CameraSensor::kCameraType) {
			slambench::io::CameraSensor * rgb_sensor = dynamic_cast<slambench::io::CameraSensor *> (sensor) ;
			std::cout << "Sensor " <<  (uint32_t)sensor->Index << ": " << " (" << count << " frames)"  << std::endl
					 << "    Type: " <<  sensor->GetType() << std::endl
					 << "    FrameFormat: " <<  rgb_sensor->FrameFormat << std::endl
					 << "    PixelFormat: " <<  slambench::io::pixelformat::TypeOf(rgb_sensor->PixelFormat) << std::endl
				 << "    Desc: " <<  sensor->Description.c_str()  << std::endl;

		} else {
			std::cout << "Sensor " <<  (uint32_t)sensor->Index << ": " << " (" << count << " frames)" << std::endl
				 << "    Type: " <<  sensor->GetType() << std::endl
				 << "    Desc: " <<  sensor->Description.c_str()  << std::endl;
		}
	}


	for(unsigned int index = 0; index < file.GetFrameCount(); ++index) {
		SLAMFrame *frame = file.GetFrame(index);
		printFrame(frame,index);
	}
}

int main(int , char **argv) {
	SLAMFile file;
	Deserialise(argv[1], file);
	Check(file);
	
	return 0;
}

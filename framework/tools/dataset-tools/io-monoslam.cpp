/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
#include <io/SLAMFile.h>
#include <io/sensor/Sensor.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>
#include <io/SLAMFrame.h>
#include <io/sensor/GroundTruthSensor.h>
#include <Eigen/Eigen>

#include <iomanip>
#include <sstream>
#include <string>
#include <regex>
#include <fstream>

#include <cmath>

#include <unistd.h>
using namespace slambench::io;

CameraSensor *GetGreySensor(const Sensor::pose_t &pose, const CameraSensor::intrinsics_t &intrinsics) {
	CameraSensor *sensor = new CameraSensor(sensortype::Grey);
	sensor->Index = 0;
	sensor->Width = 320;
	sensor->Height = 240;
	sensor->FrameFormat = frameformat::Raster;
	sensor->PixelFormat = pixelformat::G_I_8;
	
	sensor->CopyPose(pose);	
	sensor->CopyIntrinsics(intrinsics);
	
	return sensor;
}


void AddSensors(SLAMFile &file, bool Depth, bool RGB, bool Grey, bool GT) {

	// TODO This information should come from the dataset !!

	Sensor::pose_t pose_depth = Eigen::Matrix4f::Identity();
	Sensor::pose_t pose = Eigen::Matrix4f::Identity();
	
	CameraSensor::intrinsics_t intrinsics;
	intrinsics[0] = 1.0;
	intrinsics[1] = 1.0;
	intrinsics[2] = 0.5;
	intrinsics[3] = 0.5;

	DepthSensor::intrinsics_t intrinsics_depth;
	intrinsics_depth[0] = 0.751875;
	intrinsics_depth[1] = 1.0;
	intrinsics_depth[2] = 0.5;
	intrinsics_depth[3] = 0.5;

	DepthSensor::disparity_params_t disparity_params = {0.0002,0.0};
	DepthSensor::disparity_type_t disparity_type = DepthSensor::affine_disparity;
	
	int idx = 0;
	
	if(Grey) {
		CameraSensor *grey_sensor = GetGreySensor(pose, intrinsics);
		grey_sensor->Index = idx++;
		file.Sensors.AddSensor(grey_sensor);
	}
	
}


bool GetFrame(const std::string &dirname, SLAMFile &file, int frame_no) {
	// two frames to add: one for rgb and one for depth
	Sensor *grey_sensor = file.Sensors.GetSensor(sensortype::Grey);
	
	if(grey_sensor) {
		ImageFileFrame *rgb_frame = new ImageFileFrame();
		rgb_frame->FrameSensor = grey_sensor;
		rgb_frame->Timestamp.S = frame_no;
		
		std::stringstream frame_name;
		frame_name << dirname << "/rawoutput" << std::setw(4) << std::setfill('0') << frame_no << ".pgm";
		rgb_frame->Filename = frame_name.str();
		
		if(access(rgb_frame->Filename.c_str(), F_OK) < 0) {
			printf("No greyscale image for frame %u (%s)\n", frame_no, frame_name.str().c_str());
			perror("");
			return false;
		}
		
		file.AddFrame(rgb_frame);
	}
	return true;
}

bool AddFrames(const std::string &dirname, SLAMFile &file) {
	int frame_no = 0;
	
	while(GetFrame(dirname, file, frame_no)) {
		printf("\rRead frame %u  ", frame_no);
		frame_no++;
	}
	printf("\n");
	
	return true;
}


void frame_callback(int idx, int total) {
	printf("\r");
	
	// print progress bar
	printf("[");
	const int width = 50;
	float blocks = width * ((float)idx / total);
	
	for(int i = 0; i < blocks; ++i) {
		printf("#");
	}
	for(int i = blocks; i < width; ++i) {
		printf(" ");
	}
	printf("] ");
	
	printf("%u / %u", idx, total);
	fflush(stdout);
}
bool Serialise(const std::string &filename, SLAMFile &file) {
	return SLAMFile::Write(filename, file, frame_callback);
}



int main(int argc, char **argv) {
	
	SLAMFile slamfile;
	AddSensors(slamfile, 0, 0,1, 0);
	
	if(!AddFrames(argv[1], slamfile)) {
		fprintf(stderr, "Failed to add frames\n");
		return 1;
	}
	
	printf("Writing output\n");
	if(!Serialise(argv[2], slamfile)) {
		fprintf(stderr, "\nFailed to write output\n");
		return 1;
	}
	
	printf("\nSerialisation succeeded\n");
	
	return 0;
}
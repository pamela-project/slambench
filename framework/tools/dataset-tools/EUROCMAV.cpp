/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "include/EUROCMAV.h"

#include <io/SLAMFile.h>
#include <io/sensor/Sensor.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/IMUSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/SLAMFrame.h>

#include <cstdio>
#include <dirent.h>
#include <cstring>

#include <vector>
#include <string>
#include <fstream>

#include <iostream>

#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>


#include "io/sensor/SensorCollection.h"
#include "io/sensor/CameraSensor.h"
#include "io/SLAMFile.h"
#include "io/SLAMFrame.h"


using namespace slambench::io ;

bool loadIMUData(const std::string &dirname , slambench::io::IMUSensor * IMU_sensor , slambench::io::SLAMFile *file) {

	std::string line;

	boost::smatch match;
	std::ifstream infile(dirname + "/" + "data.csv");

	while (std::getline(infile, line)){

		if (line.size() == 0) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^([0-9]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+)\\s*$"))) {

			uint64_t timestamp = strtol(std::string(match[1]).c_str(), nullptr, 10);

			int timestampS  = timestamp / 1000000000;
			int timestampNS = timestamp % 1000000000;

			float gx =  std::stof(match[2]);
			float gy =  std::stof(match[3]);
			float gz =  std::stof(match[4]);
			float ax =  std::stof(match[5]);
			float ay =  std::stof(match[6]);
			float az =  std::stof(match[7]);

			slambench::io::SLAMInMemoryFrame *IMU_frame = new slambench::io::SLAMInMemoryFrame();
			IMU_frame->FrameSensor = IMU_sensor;
			IMU_frame->Timestamp.S  = timestampS;
			IMU_frame->Timestamp.Ns = timestampNS;
			IMU_frame->Data = malloc(IMU_sensor->GetFrameSize(IMU_frame));

			((float*)IMU_frame->Data)[0] = gx;
			((float*)IMU_frame->Data)[1] = gy;
			((float*)IMU_frame->Data)[2] = gz;

			((float*)IMU_frame->Data)[3] = ax;
			((float*)IMU_frame->Data)[4] = ay;
			((float*)IMU_frame->Data)[5] = az;

			file->AddFrame(IMU_frame);

		} else {
			std::cerr << "Unknown line:" << line << std::endl;
			return false;
		}


	}
	return true;
}

bool loadGTData(const std::string &dirname , slambench::io::Sensor *gt_sensor , slambench::io::SLAMFile *file) {

	std::string line;

	boost::smatch match;
	std::ifstream infile(dirname + "/" + "data.csv");

	while (std::getline(infile, line)){

		if (line.size() == 0) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^([0-9]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+),([-0-9.]+)\\s*$"))) {

			uint64_t timestamp = strtol(std::string(match[1]).c_str(), nullptr, 10);
			int timestampS  = timestamp / 1000000000;
			int timestampNS = timestamp % 1000000000;

			float p_RS_R_x = std::stof(match[2]) ;  //  [m],
			float p_RS_R_y = std::stof(match[3]) ;  //  [m],
			float p_RS_R_z = std::stof(match[4]) ;  //  [m],

			float q_RS_w =  std::stof(match[5]);  // [],
			float q_RS_x =  std::stof(match[6]);  // [],
			float q_RS_y =  std::stof(match[7]);  // [],
			float q_RS_z =  std::stof(match[8]);  // [],

			//float v_RS_R_x  =  std::stof(match[9]) ; // [m s^-1]
			//float v_RS_R_y  =  std::stof(match[10]) ; // [m s^-1]
			//float v_RS_R_z  =  std::stof(match[11]) ; // [m s^-1]

			//float b_w_RS_S_x  =  std::stof(match[11]) ; // [rad s^-1],
			//float b_w_RS_S_y  =  std::stof(match[12]) ; // [rad s^-1],
			//float b_w_RS_S_z  =  std::stof(match[13]) ; // [rad s^-1],

			//float b_a_RS_S_x  =  std::stof(match[14]) ; // [m s^-2]
			//float b_a_RS_S_y  =  std::stof(match[15]) ; // [m s^-2]
			//float b_a_RS_S_z  =  std::stof(match[16]) ; // [m s^-2]

			Eigen::Matrix3f rotationMat = Eigen::Quaternionf(q_RS_w,q_RS_x,q_RS_y,q_RS_z).toRotationMatrix();
			Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
			pose.block(0,0,3,3) = rotationMat;
			pose.block(0,3,3,1) << p_RS_R_x , p_RS_R_y , p_RS_R_z;


			slambench::io::SLAMInMemoryFrame *gt_frame = new slambench::io::SLAMInMemoryFrame();
			gt_frame->FrameSensor = gt_sensor;
			gt_frame->Timestamp.S  = timestampS;
			gt_frame->Timestamp.Ns = timestampNS;
			gt_frame->Data = malloc(gt_sensor->GetFrameSize(gt_frame));


			memcpy(gt_frame->Data,pose.data(),gt_sensor->GetFrameSize(gt_frame));

			file->AddFrame(gt_frame);

		} else {
			std::cerr << "Unknown line:" << line << std::endl;
			return false;
		}


	}
	return true;
}



SLAMFile* EUROCMAVReader::GenerateSLAMFile ()  {


	// first, see what sensors we have

	std::vector<std::string> sensor_directories;
	std::string input_dir = input;
	DIR *dir = opendir(input_dir.c_str());
	dirent *pdir;
	while((pdir = readdir(dir)) != nullptr) {
		if(pdir->d_type == DT_DIR) {
			if(strcmp(pdir->d_name, ".") == 0) {
				continue;
			}
			if(strcmp(pdir->d_name, "..") == 0) {
				continue;
			}

			sensor_directories.push_back(pdir->d_name);
		}
	}

	slambench::io::SLAMFile *slamfile = new slambench::io::SLAMFile();

	for(auto &dirname : sensor_directories) {
		// try and get sensor.yaml file
		std::string cam_dirname = input_dir + "/" + dirname;
		std::string filename = cam_dirname + "/sensor.yaml";
		YAML::Node sensor = YAML::LoadFile(filename.c_str());

		// check sensor type
		std::string sensor_type = sensor["sensor_type"].as<std::string>();
		if(sensor_type == "camera" and this->stereo) {
			std::cerr << "Found sensor type " << sensor_type << " from directory " << dirname << std::endl;


			// Create a Grey sensor

			slambench::io::CameraSensor *camsensor = new slambench::io::CameraSensor(dirname);
			camsensor->Index = slamfile->Sensors.size();
			camsensor->Rate = sensor["rate_hz"].as<float>();
			camsensor->Description = sensor["comment"].as<std::string>();
			camsensor->FrameFormat = slambench::io::frameformat::Raster;
			camsensor->PixelFormat = slambench::io::pixelformat::G_I_8;

			camsensor->Width = sensor["resolution"][0].as<int>();
			camsensor->Height = sensor["resolution"][1].as<int>();


			camsensor->Intrinsics[0] = sensor["intrinsics"][0].as<float>() / camsensor->Width;
			camsensor->Intrinsics[1] = sensor["intrinsics"][1].as<float>() / camsensor->Height;
			camsensor->Intrinsics[2] = sensor["intrinsics"][2].as<float>() / camsensor->Width;
			camsensor->Intrinsics[3] = sensor["intrinsics"][3].as<float>() / camsensor->Height;
			if (sensor["distortion_model"].as<std::string>() == "radial-tangential") {
				camsensor->DistortionType = slambench::io::CameraSensor::distortion_type_t::RadialTangential;

				camsensor->RadialTangentialDistortion[0] = sensor["distortion_coefficients"][0].as<float>();
				camsensor->RadialTangentialDistortion[1] = sensor["distortion_coefficients"][1].as<float>();
				camsensor->RadialTangentialDistortion[2] = sensor["distortion_coefficients"][2].as<float>();
				camsensor->RadialTangentialDistortion[3] = sensor["distortion_coefficients"][3].as<float>();
				camsensor->RadialTangentialDistortion[4] = 0;

			} else {
				std::cerr << "Unsupported distortion type for Eurocmav." << std::endl;
				exit(1);
			}

			std::cout << "pose is ... " << std::endl;

			Eigen::Matrix4f pose;


			for(int i = 0; i < 16; ++i) {
				int y = i % 4;
				int x = i / 4;
				std::cout << " " << sensor["T_BS"]["data"][i].as<float>();
				pose(x,y) = sensor["T_BS"]["data"][i].as<float>();
				if ((i+1)%4 == 0) std::cout  << std::endl;
			}

			camsensor->CopyPose(pose);
			std::cout  << std::endl;

			slamfile->Sensors.AddSensor(camsensor);


			// Create a RGB equivalent sensor

			slambench::io::CameraSensor *rgbsensor = new slambench::io::CameraSensor(dirname + "clone");
			rgbsensor->Index = slamfile->Sensors.size();
			rgbsensor->Description = "RGB clone from " + sensor["comment"].as<std::string>();
			rgbsensor->Rate = sensor["rate_hz"].as<float>();
			rgbsensor->FrameFormat = slambench::io::frameformat::Raster;
			rgbsensor->PixelFormat = slambench::io::pixelformat::RGB_III_888;
			rgbsensor->Width = sensor["resolution"][0].as<int>();
			rgbsensor->Height = sensor["resolution"][1].as<int>();
			rgbsensor->Intrinsics[0] = sensor["intrinsics"][0].as<float>() / rgbsensor->Width;
			rgbsensor->Intrinsics[1] = sensor["intrinsics"][1].as<float>() / rgbsensor->Height;
			rgbsensor->Intrinsics[2] = sensor["intrinsics"][2].as<float>() / rgbsensor->Width;
			rgbsensor->Intrinsics[3] = sensor["intrinsics"][3].as<float>() / rgbsensor->Height;
			rgbsensor->CopyPose(pose);
			if (sensor["distortion_model"].as<std::string>() == "radial-tangential") {
				rgbsensor->DistortionType = slambench::io::CameraSensor::distortion_type_t::RadialTangential;

				rgbsensor->RadialTangentialDistortion[0] = sensor["distortion_coefficients"][0].as<float>();
				rgbsensor->RadialTangentialDistortion[1] = sensor["distortion_coefficients"][1].as<float>();
				rgbsensor->RadialTangentialDistortion[2] = sensor["distortion_coefficients"][2].as<float>();
				rgbsensor->RadialTangentialDistortion[3] = sensor["distortion_coefficients"][3].as<float>();
				rgbsensor->RadialTangentialDistortion[4] = 0;

			} else {
				std::cerr << "Unsupported distortion type for Eurocmav." << std::endl;
				exit(1);
			}


			if (this->rgb) {
				slamfile->Sensors.AddSensor(rgbsensor);
			}


			// now, load frames
			dir = opendir((cam_dirname + "/data/").c_str());
			while((pdir = readdir(dir)) != nullptr) {
				if(pdir->d_type == DT_REG) {

					// Add the original Grey Image

					slambench::io::ImageFileFrame *frame = new slambench::io::ImageFileFrame();
					frame->FrameSensor = camsensor;
					frame->Filename = cam_dirname + "/data/" + pdir->d_name;

					uint64_t timestamp = strtol(pdir->d_name, nullptr, 10);
					frame->Timestamp.S = timestamp / 1000000000;
					frame->Timestamp.Ns = timestamp % 1000000000;

					slamfile->AddFrame(frame);

					if (this->rgb) {
						// Add the clone RGB
						slambench::io::ImageFileFrame *rgb_frame = new slambench::io::ImageFileFrame();
						rgb_frame->FrameSensor = rgbsensor;
						rgb_frame->Filename = cam_dirname + "/data/" + pdir->d_name;
						rgb_frame->Timestamp.S = timestamp / 1000000000;
						rgb_frame->Timestamp.Ns = timestamp % 1000000000;
						slamfile->AddFrame(rgb_frame);
					}
				}
			}
		} else if (sensor_type == "imu" and this->imu) {

			std::cerr << "Found sensor type " << sensor_type << " from directory " << dirname << std::endl;
			slambench::io::IMUSensor *accsensor = new slambench::io::IMUSensor(dirname);
			accsensor->Index = slamfile->Sensors.size();
			accsensor->Description = sensor["comment"].as<std::string>();
			accsensor->Rate = sensor["rate_hz"].as<float>();

			accsensor->GyroscopeNoiseDensity = sensor["gyroscope_noise_density"].as<float>();
			accsensor->GyroscopeDriftNoiseDensity = 4.0e-6;
			accsensor->GyroscopeBiasDiffusion = sensor["gyroscope_random_walk"].as<float>();
			accsensor->GyroscopeSaturation   =   7.8;

			accsensor->AcceleratorNoiseDensity = sensor["accelerometer_noise_density"].as<float>();
			accsensor->AcceleratorDriftNoiseDensity = 4.0e-5;
			accsensor->AcceleratorBiasDiffusion = sensor["accelerometer_random_walk"].as<float>();
			accsensor->AcceleratorSaturation = 176.0;



			double tau= 3600.0 ; // # reversion time constant, currently not in use [s]
			double g= 9.81007 ; // # Earth's acceleration due to gravity [m/s^2]

			Eigen::Vector3d a0 = {0.0, 0.0, 0.0};// # Accelerometer bias [m/s^2]



			Eigen::Matrix4f pose;


					for(int i = 0; i < 16; ++i) {
						int y = i % 4;
						int x = i / 4;
						std::cout << " " << sensor["T_BS"]["data"][i].as<float>();
						pose(x,y) = sensor["T_BS"]["data"][i].as<float>();
						if ((i+1)%4 == 0) std::cout  << std::endl;
					}

					accsensor->CopyPose(pose);



			slamfile->Sensors.AddSensor(accsensor);

			if (not loadIMUData(cam_dirname , accsensor, slamfile)) {delete slamfile; return nullptr;}
		} else if (sensor_type == "visual-inertial"  and this->gt ) {
			std::cerr << "Found sensor type " << sensor_type << " from directory " << dirname << std::endl;

			slambench::io::GroundTruthSensor *gt_sensor = new slambench::io::GroundTruthSensor(dirname);
			gt_sensor->Index = slamfile->Sensors.size();
			gt_sensor->Description = "Ground Truth";
			slamfile->Sensors.AddSensor(gt_sensor);







			if (not loadGTData(cam_dirname , gt_sensor, slamfile)) {delete slamfile; return nullptr;}
		} else {
			std::cerr << "Unknown sensor type " << sensor_type << " from directory " << dirname << std::endl;
		}
	}

	return slamfile;

}



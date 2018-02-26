/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "../dataset-tools/include/SVO.h"

#include <io/SLAMFile.h>
#include <io/sensor/Sensor.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/AccelerometerSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/SLAMFrame.h>

#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

#include <cmath>
#include <Eigen/Eigen>

#include <iostream>

#include <unistd.h>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp>


using namespace slambench::io ;


static const CameraSensor::intrinsics_t svo_grey   = { 0.419547872, 0.657291667, 0.5, 0.5 }; // ATAN
static const float translation[] = {0.1131, 0.1131, 2.0}; // x, y, z
static const float rotation[] = {0.0, 0.9675388, 0.2527226, 0.0}; // w, x, y, z
//static const CameraSensor::intrinsics_t svo_grey   = { 315.5, 315.5, 376.0, 240.0 }; // Pinhole


bool loadSVOGreyData(const std::string &dirname , SLAMFile &file, const Sensor::pose_t &pose, const CameraSensor::intrinsics_t &intrinsics) {

	CameraSensor *grey_sensor = new CameraSensor("Grey", CameraSensor::kCameraType);
	grey_sensor->Index = 0;
	grey_sensor->Width = 752;
	grey_sensor->Height = 480;
	grey_sensor->FrameFormat = frameformat::Raster;
	grey_sensor->PixelFormat = pixelformat::G_I_8;
	grey_sensor->Description = "Grey";

	grey_sensor->CopyPose(pose);
	grey_sensor->CopyIntrinsics(intrinsics);
	grey_sensor->Index =file.Sensors.size();
	file.Sensors.AddSensor(grey_sensor);


    for (int frame_no = 2; frame_no < 188; frame_no++) {
		  ImageFileFrame *grey_frame = new ImageFileFrame();
		  grey_frame->FrameSensor = grey_sensor;
        grey_frame->Timestamp.S = frame_no - 2;

		  std::stringstream frame_name;
        frame_name << dirname << "/img/frame_" << std::setw(6) << std::setfill('0')
            << frame_no << "_0.png";
		  grey_frame->Filename = frame_name.str();

		  if(access(grey_frame->Filename.c_str(), F_OK) < 0) {
            printf("No grey image for frame %u (%s)\n", frame_no, frame_name.str().c_str());
		    return false;
		  }

		  file.AddFrame(grey_frame);
	}
	return true;
}

bool loadSVOGroundTruthData(const std::string &dirname , SLAMFile &file) {

    GroundTruthSensor *gt_sensor = new GroundTruthSensor("GT");
	gt_sensor->Index = file.Sensors.size();
	gt_sensor->Description = "GroundTruthSensor";
	file.Sensors.AddSensor(gt_sensor);

	if(!gt_sensor) {
		std::cout << "gt sensor not found..." << std::endl;
		return false;
	} else {
		std::cout << "gt sensor created..." << std::endl;
	}


	std::string line;
    std::vector<std::string> results;
    std::ifstream infile(dirname + "/" + "trajectory_nominal.txt");

    int timestampS = 0;
	while (std::getline(infile, line)){
        if (line.size() == 0)
			continue;
        boost::split(results, line, boost::is_any_of(" "));
        //int timestampS = std::stoi(results[0]);
        float tx = std::stof(results[1]);
        float ty = std::stof(results[2]);
        float tz = std::stof(results[3]);

        float QX = std::stof(results[4]);
        float QY = std::stof(results[5]);
        float QZ = std::stof(results[6]);
        float QW = std::stof(results[7]);

        // the raw ground truth data are not normalized, we need normalized quaternions
        float Q_length = std::sqrt(std::pow(QX, 2) + std::pow(QY, 2) +
            std::pow(QZ, 2) + std::pow(QW, 2));
        QX /= Q_length;
        QY /= Q_length;
        QZ /= Q_length;
        QW /= Q_length;

		  Eigen::Matrix3f rotationMat = Eigen::Quaternionf(QW,QX,QY,QZ).toRotationMatrix();
		  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

		  pose.block(0,0,3,3) = rotationMat;
		  pose.block(0,3,3,1) << tx , ty , tz;

		  SLAMInMemoryFrame *gt_frame = new SLAMInMemoryFrame();
		  gt_frame->FrameSensor = gt_sensor;
     		  gt_frame->Timestamp.S  = timestampS++;
		  gt_frame->Data = malloc(gt_frame->GetSize());

		  	memcpy(gt_frame->Data,pose.data(),gt_frame->GetSize());

		  file.AddFrame(gt_frame);
    }
    return true;
}





SLAMFile* SVOReader::GenerateSLAMFile () {


	std::string dirname = input;


	SLAMFile * slamfilep = new SLAMFile();
	SLAMFile & slamfile  = *slamfilep;

	Sensor::pose_t pose = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f rotationMat = Eigen::Quaternionf(rotation[0], rotation[1], rotation[2], rotation[3]).toRotationMatrix();
    pose.block(0,0,3,3) = rotationMat;
    pose.block(0,3,3,1) << translation[0], translation[1], translation[2];

	/**
	 * load Grey
	 */

	if(!loadSVOGreyData(dirname, slamfile,pose,svo_grey)) {
		std::cout << "Error while loading Grey information." << std::endl;
		delete slamfilep;
		return nullptr;

	}

	/**
	 * load GT
	 */
	if(!loadSVOGroundTruthData(dirname, slamfile)) {
		std::cout << "Error while loading gt information." << std::endl;
		delete slamfilep;
		return nullptr;

	}

	return slamfilep;
	}







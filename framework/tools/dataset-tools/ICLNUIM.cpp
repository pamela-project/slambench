/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "./include/ICLNUIM.h"

#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/PointCloudSensor.h>
#include <io/format/PointCloud.h>
#include <Eigen/Eigen>


#include <boost/regex.hpp>

#include <iostream>
#include <fstream>
#include <Eigen/Core>

using namespace slambench::io ;


DepthSensor *GetDepthSensor(const Sensor::pose_t &pose, const DepthSensor::intrinsics_t &intrinsics, const DepthSensor::disparity_params_t &dparams, const DepthSensor::disparity_type_t &dtype) {
	DepthSensor *sensor = new DepthSensor("Depth");
	sensor->Index = 0;
	sensor->Rate = 1;
	sensor->Width = 640;
	sensor->Height = 480;
	sensor->FrameFormat = frameformat::Raster;
	sensor->PixelFormat = pixelformat::D_I_16;
	sensor->DisparityType = dtype;
	sensor->CopyPose(pose);
	sensor->CopyIntrinsics(intrinsics);
	sensor->CopyDisparityParams(dparams);

	return sensor;
}

CameraSensor *GetRGBSensor(const Sensor::pose_t &pose, const CameraSensor::intrinsics_t &intrinsics) {
	CameraSensor *sensor = new CameraSensor("RGB", CameraSensor::kCameraType);
	sensor->Index = 0;
	sensor->Rate = 1;
	sensor->Width = 640;
	sensor->Height = 480;
	sensor->FrameFormat = frameformat::Raster;
	sensor->PixelFormat = pixelformat::RGB_III_888;

	sensor->CopyPose(pose);
	sensor->CopyIntrinsics(intrinsics);

	return sensor;
}

CameraSensor *GetGreySensor(const Sensor::pose_t &pose, const CameraSensor::intrinsics_t &intrinsics) {
	CameraSensor *sensor = new CameraSensor("Grey", CameraSensor::kCameraType);
	sensor->Index = 0;
	sensor->Rate = 1;
	sensor->Width = 640;
	sensor->Height = 480;
	sensor->FrameFormat = frameformat::Raster;
	sensor->PixelFormat = pixelformat::G_I_8;

	sensor->CopyPose(pose);
	sensor->CopyIntrinsics(intrinsics);

	return sensor;
}

GroundTruthSensor *GetGTSensor(const Sensor::pose_t &pose) {
	GroundTruthSensor *sensor = new GroundTruthSensor("GroundTruth");
	sensor->Index = 0;
	sensor->Rate = 1;
	sensor->Description = "Ground Truth";
	sensor->CopyPose(pose);
	return sensor;
}

void ICLNUIMReader::AddSensors(SLAMFile &file) {

	// TODO This information should come from the dataset !!

	Sensor::pose_t pose_depth = Eigen::Matrix4f::Identity();
	Sensor::pose_t pose = Eigen::Matrix4f::Identity();

	CameraSensor::intrinsics_t intrinsics;
	intrinsics[0] = 0.751875;
	intrinsics[1] = - 1.0;
	if (this->positive_focal) intrinsics[1] = 1.0; // TODO : This is actually -1, bug .. no.
	intrinsics[2] = 0.4992185;
	intrinsics[3] = 0.4989583;

	DepthSensor::intrinsics_t intrinsics_depth;
	intrinsics_depth[0] = 0.751875;
	intrinsics_depth[1] = - 1.0;
	if (this->positive_focal) intrinsics_depth[1] = 1.0; // TODO : This is actually -1, bug .. no.
	intrinsics_depth[2] = 0.4992185;
	intrinsics_depth[3] = 0.4989583;

	DepthSensor::disparity_params_t disparity_params = {0.001,0.0};
	DepthSensor::disparity_type_t disparity_type = DepthSensor::affine_disparity;

	int idx = 0;

	if(this->rgb) {
		this->rgb_sensor = GetRGBSensor(pose, intrinsics);
		this->rgb_sensor->Index = idx++;
		file.Sensors.AddSensor(this->rgb_sensor);
	}

	if(this->depth) {
		this->depth_sensor = GetDepthSensor(pose_depth, intrinsics_depth,disparity_params,disparity_type);
		this->depth_sensor->Index = idx++;
		file.Sensors.AddSensor(this->depth_sensor);
	}

	if(this->grey) {
		this->grey_sensor = GetGreySensor(pose, intrinsics);
		this->grey_sensor->Index = idx++;
		file.Sensors.AddSensor(this->grey_sensor);
	}

	if(this->gt) {
		this->gt_sensor = GetGTSensor(pose);
		this->gt_sensor->Index = idx++;
		file.Sensors.AddSensor(this->gt_sensor);
	}
}

static void undistort_frame(slambench::io::SLAMFileFrame *frame, void *data) {
	uint16_t *depthMap = (uint16_t*)data;

	uint32_t w = ((slambench::io::CameraSensor*)frame->FrameSensor)->Width;
	uint32_t h = ((slambench::io::CameraSensor*)frame->FrameSensor)->Height;

	float u0 = 319.50;
	float v0 = 239.50;
	float fx = 481.20;
	float fy = -480.00;

    for (uint32_t v = 0; v < h; v++) {
        for (uint32_t u = 0; u < w; u++) {
            double u_u0_by_fx = (u - u0) / fx;
            double v_v0_by_fy = (v - v0) / fy;

            depthMap[u + v * w] = depthMap[u + v * w] / std::sqrt(u_u0_by_fx * u_u0_by_fx
                                                                  + v_v0_by_fy * v_v0_by_fy
                                                                  + 1);
        }
    }
}

static float3 normalise(const float3 &input) {
	float3 output = input;
	float magnitude = std::abs(std::sqrt((output.x * output.x) + (output.y * output.y) + (output.z * output.z)));
	if (output.x != 0)	output.x /= magnitude;
	if (output.y != 0)	output.y /= magnitude;
	if (output.z != 0)	output.z /= magnitude;
	return output;
}


void printMat (const Eigen::Matrix4f& mat, std::string color = "\033[0m") {
	std::cout << std::fixed << std::setprecision(3) << color
	<< std::setw(6)  << mat(0,0) << " "
	<< std::setw(6)  << mat(0,1) << " "
	<< std::setw(6)  << mat(0,2) << " "
	<< std::setw(6)  << mat(0,3) << " "
	<< std::setw(6)  << mat(1,0) << " "
	<< std::setw(6)  << mat(1,1) << " "
	<< std::setw(6)  << mat(1,2) << " "
	<< std::setw(6)  << mat(1,3) << " "
	<< std::setw(6)  << mat(2,0) << " "
	<< std::setw(6)  << mat(2,1) << " "
	<< std::setw(6)  << mat(2,2) << " "
	<< std::setw(6)  << mat(2,3) << " "
	<< std::setw(6)  << mat(3,0) << " "
	<< std::setw(6)  << mat(3,1) << " "
	<< std::setw(6)  << mat(3,2) << " "
	<< std::setw(6)  << mat(3,3) << " " << "\033[0m"


	<< std::endl;

}

bool FillPose(const std::string &filename, GroundTruthSensor::pose_t &pose, bool positive_focal) {
  boost::regex key_regex("^([a-z_]+)");
  boost::regex value_regex ("([-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?), ([-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?), ([-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?)");

	std::ifstream istream(filename);

	// find the important features for calculating the pose
	std::map<std::string, float3> kvs;
	std::string line;

	//ksstd::cout << "" << std::endl;
	while(std::getline(istream, line)) {

		//std::cout << line << std::endl;

		boost::cmatch match;
		if(!boost::regex_search(line.c_str(), match, key_regex)) {
			continue;
		}

		std::string key = match.str(0);

		if(!boost::regex_search(line.c_str(), match, value_regex)) {
			if (key != "cam_angle") {
				std::cout << "Error reading this line :" << line << std::endl;
				exit(1);
			}

			continue;
		}

		kvs[key].x = strtof(match.str(1).c_str(), nullptr);
		kvs[key].y = strtof(match.str(3).c_str(), nullptr);
		kvs[key].z = strtof(match.str(5).c_str(), nullptr);
	}
    //
	//std::cout << "" << std::endl;
	//for (auto item : kvs) {
	//	std::cout << item.first  << "=" << "[" <<  item.second.x << ", " <<  item.second.y << ", " <<  item.second.z << "]"  << std::endl;
	//}
	//std::cout << "" << std::endl;

	// row 3
	float3 los = normalise(kvs["cam_dir"]);
	float3 up  = normalise(kvs["cam_up"]);
	float3 right = normalise(kvs["cam_right"]);

	// std::cout << std::setw(13)  << los.x   << std::setw(13)  << los.y   << std::setw(13)   << los.z
	// 		   << std::setw(13) << up.x    << std::setw(13)  << up.y    << std::setw(13)  << up.z
	// 		   << std::setw(13) << right.x  << std::setw(13) << right.y  << std::setw(13) << right.z  << std::endl;
    //
    pose(0,0) = right.x; pose(0,1) = right.y; pose(0,2) = right.z; pose(0,3) = kvs["cam_pos"].x;
    pose(1,0) = up.x;    pose(1,1) = up.y;    pose(1,2) = up.z;    pose(1,3) = kvs["cam_pos"].y;
	pose(2,0) = los.x;   pose(2,1) = los.y;   pose(2,2) = los.z;   pose(2,3) = kvs["cam_pos"].z;
	pose(3,0) = 0.0;     pose(3,1) = 0.0;     pose(3,2) = 0.0;     pose(3,3) = 1.0;

	if (positive_focal) {

		// First The ground truth is not align with the point cloud we need to flip
		pose(0, 3) *= -1.0; //Pos


		 // Second the ground truth is looking to the wrong Z direction
		 static const GroundTruthSensor::pose_t  origin = pose ;
		 pose = origin.inverse()     * pose;
		 pose(0, 2) *= -1.0; //Rot
		 pose(1, 2) *= -1.0; //Rot
		 pose(2, 0) *= -1.0; //Rot
		 pose(2, 1) *= -1.0; //Rot
		 pose = origin    * pose;


		 // Finally the camera is upside down !
		 pose = origin.inverse()     * pose;
		 pose.block<2,3>(0,0) = -pose.block<2,3>(0,0);
		 pose = origin    * pose;


	}

	return true;

}

bool ICLNUIMReader::GetFrame(const std::string &dirname, SLAMFile &file, int frame_no) {
	// two frames to add: one for rgb and one for depth

	const int frame_rate = 25;
	double frame_time = 1.0 / frame_rate;

	uint64_t total_ns = frame_time * frame_no * 1000000000;
	slambench::TimeStamp ts;
	ts.S = total_ns / 1000000000;
	ts.Ns = total_ns % 1000000000;

	if(rgb_sensor) {
		ImageFileFrame *rgb_frame = new ImageFileFrame();
		rgb_frame->FrameSensor = rgb_sensor;
		
		rgb_frame->Timestamp = ts;

		std::stringstream frame_name;
		frame_name << dirname << "/scene_00_" << std::setw(4) << std::setfill('0') << frame_no << ".png";
		rgb_frame->Filename = frame_name.str();

		if(access(rgb_frame->Filename.c_str(), F_OK) < 0) {
			printf("No RGB image for frame %u (%s)\n", frame_no, frame_name.str().c_str());
			return false;
		}

		file.AddFrame(rgb_frame);
	}
	if(grey_sensor) {
		ImageFileFrame *rgb_frame = new ImageFileFrame();
		rgb_frame->FrameSensor = grey_sensor;
		rgb_frame->Timestamp = ts;

		std::stringstream frame_name;
		frame_name << dirname << "/scene_00_" << std::setw(4) << std::setfill('0') << frame_no << ".png";
		rgb_frame->Filename = frame_name.str();

		if(access(rgb_frame->Filename.c_str(), F_OK) < 0) {
			printf("No Grey image for frame %u (%s)\n", frame_no, frame_name.str().c_str());
			perror("");
			return false;
		}

		file.AddFrame(rgb_frame);
	}
	if(depth_sensor) {
		TxtFileFrame *depth_frame = new TxtFileFrame();
		depth_frame->FrameSensor = depth_sensor;
		depth_frame->Timestamp = ts;
		depth_frame->InputPixelFormat = pixelformat::D_F_32;
		depth_frame->ProcessCallback = undistort_frame;

		std::stringstream frame_name;
		frame_name << dirname << "/scene_00_" << std::setw(4) << std::setfill('0') << frame_no << ".depth";
		depth_frame->Filename = frame_name.str();

		if(access(depth_frame->Filename.c_str(), F_OK) < 0) {
			printf("No depth image for frame %u (%s)\n", frame_no, frame_name.str().c_str());
			perror("");
			return false;
		}

		file.AddFrame(depth_frame);
	}

	if(gt_sensor) {
		SLAMInMemoryFrame *frame = new SLAMInMemoryFrame();
		frame->FrameSensor = gt_sensor;
		frame->Timestamp = ts;
		frame->Data = malloc(frame->GetSize());
		bzero(frame->Data, frame->GetSize());

		std::stringstream frame_name;
		frame_name << dirname << "/scene_00_" << std::setw(4) << std::setfill('0') << frame_no << ".txt";

		if(access(frame_name.str().c_str(), F_OK) < 0) {
			printf("No metadata for frame %u (%s)\n", frame_no, frame_name.str().c_str());
			perror("");
			return false;
		}

		FillPose(frame_name.str(), *((GroundTruthSensor::pose_t*)frame->Data), this->positive_focal);

		file.AddFrame(frame);
	}

	return true;
}

bool ICLNUIMReader::AddFrames(const std::string &dirname, SLAMFile &file) {
	int frame_no = 0;

	while(GetFrame(dirname, file, frame_no)) {
		frame_no++;
	}

	return true;
}


// Case-insensitive string equality check
bool strieq(const char *a, const char *b) {
	if(a == nullptr || b == nullptr) {
		throw std::logic_error("Cannot handle null strings");
	}

	while(*a || *b) {
		if(*a == '\0' || *b == '\0') {
			return false;
		}

		if(tolower(*a) != tolower(*b)) {
			return false;
		}

		a++;
		b++;
	}

	return true;
}

void AddPointCloudSensor(slambench::io::SLAMFile &slamfile) {
	slambench::io::PointCloudSensor *pcd = new slambench::io::PointCloudSensor("PointCloud");
	pcd->Description = "Ground truth point cloud";
	pcd->Index = slamfile.Sensors.size();
	slamfile.Sensors.AddSensor(pcd);
}

void AddPlyFile(slambench::io::SLAMFile &slamfile, std::string plyname) {
	slambench::io::PlyReader plyreader;
	std::ifstream file(plyname.c_str());
	if(!file.good()) {
		fprintf(stderr, "Could not open PLY file\n");
		return;
	}
	auto *pointcloud = plyreader.Read(file);
	if(pointcloud == nullptr) {
		fprintf(stderr, "Could not build point cloud\n");
		return;
	}
	auto rawpointcloud = pointcloud->ToRaw();

	SLAMInMemoryFrame *pcloudframe = new SLAMInMemoryFrame();
	pcloudframe->FrameSensor = slamfile.GetSensor(PointCloudSensor::kPointCloudType);
	pcloudframe->Data = malloc(rawpointcloud.size());
	pcloudframe->SetVariableSize(rawpointcloud.size());
	memcpy(pcloudframe->Data, rawpointcloud.data(), rawpointcloud.size());
	slamfile.AddFrame(pcloudframe);
}





SLAMFile* ICLNUIMReader::GenerateSLAMFile () {
		std::cout << "Selection input file is " << this->input << std::endl;

		SLAMFile* slamfile = new SLAMFile();




			if(!(grey || rgb || depth || gt)) {
				std::cout << "No sensor required." << std::endl;
				delete slamfile;
				return nullptr;
			}

			if(plyfile.length() != 0) {
				std::cout << "Add point cloud." << std::endl;
				AddPointCloudSensor(*slamfile);
			}

			AddSensors(*slamfile);

			if(plyfile.length() != 0) {
				AddPlyFile(*slamfile, plyfile);
			}

			if(!AddFrames(input, *slamfile)) {
				std::cout << "Failed to add frames." << std::endl;
				delete slamfile;
				return nullptr;
			}


			return slamfile;

	}






/*

 Copyright (c) 2019 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "include/UZHFPV.h"
#include "include/utils/RegexPattern.h"
#include "include/utils/dataset_utils.h"
#include "include/utils/sensor_builder.h"

#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/IMUSensor.h>
#include <io/sensor/Sensor.h>

#include <dirent.h>
#include <cstring>

#include <fstream>
#include <string>
#include <vector>

#include <iostream>

#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

using namespace slambench::io;

bool loadUZHFPVGroundTruthData(const std::string &dirname,
                               SLAMFile &file,
                               GroundTruthSensor* gt_sensor) {

  std::ifstream infile(dirname + "/" + "groundtruth.txt");

  std::string line;
  boost::smatch match;

  const std::string& ts = RegexPattern::timestamp;
  const std::string& ws = RegexPattern::whitespace;
  const std::string& id = RegexPattern::integer;
  const std::string& dec = RegexPattern::decimal;
  const std::string& start = RegexPattern::start;
  const std::string& end = RegexPattern::end;

  // format: id timestamp tx ty tz qx qy qz qw
  std::string expr = start
      + id + ws        // id
      + ts  + ws       // timestamp
      + dec + ws       // tx
      + dec + ws       // ty
      + dec + ws       // tz
      + dec + ws       // qx
      + dec + ws       // qy
      + dec + ws       // qz
      + dec + end;     // qw

  boost::regex groundtruth_line = boost::regex(expr);
  boost::regex comment = boost::regex(RegexPattern::comment);

  while (std::getline(infile, line)) {
    if (line.empty()) {
      continue;
    } else if (boost::regex_match(line, match, comment)) {
      continue;
    } else if (boost::regex_match(line, match, groundtruth_line)) {

//      int identifier = std::stoi(match[1]);

      int timestampS = std::stoi(match[2]);
      int timestampNS = std::stod(match[3]) * std::pow(10, 9 - match[3].length());

      float tx = std::stof(match[4]);
      float ty = std::stof(match[5]);
      float tz = std::stof(match[6]);

      float QX = std::stof(match[7]);
      float QY = std::stof(match[8]);
      float QZ = std::stof(match[9]);
      float QW = std::stof(match[10]);

      Eigen::Matrix3f rotationMat = Eigen::Quaternionf(QW, QX, QY, QZ).toRotationMatrix();
      Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
      pose.block(0, 0, 3, 3) = rotationMat;
      pose.block(0, 3, 3, 1) << tx, ty, tz;

      auto gt_frame = new SLAMInMemoryFrame();
      gt_frame->FrameSensor = gt_sensor;
      gt_frame->Timestamp.S = timestampS;
      gt_frame->Timestamp.Ns = timestampNS;
      gt_frame->Data = malloc(gt_frame->GetSize());

      memcpy(gt_frame->Data, pose.data(), gt_frame->GetSize());

      file.AddFrame(gt_frame);

    } else {
      std::cout << "Unknown line:" << line << std::endl;
      return false;
    }
  }
  return true;
}

bool loadUZHFPVIMUData(const std::string &dirname,
                       SLAMFile &file,
                       IMUSensor *IMU_sensor) {

  std::string line;

  boost::smatch match;
  std::ifstream infile(dirname + "/" + "imu.txt");

  const std::string& ts = RegexPattern::timestamp;
  const std::string& ws = RegexPattern::whitespace;
  const std::string& id = RegexPattern::integer;
  const std::string& dec = RegexPattern::decimal;
  const std::string& start = RegexPattern::start;
  const std::string& end = RegexPattern::end;

  // format: timestamp ang_vel_x ang_vel_y ang_vel_z lin_acc_x lin_acc_y lin_acc_z
  std::string expr = start
      + id  + ws             // id
      + ts  + ws             // timestamp
      + dec + ws             // ang_vel_x
      + dec + ws             // ang_vel_y
      + dec + ws             // ang_vel_z
      + dec + ws             // lin_acc_x
      + dec + ws             // lin_acc_y
      + dec + end;           // lin_acc_z

  boost::regex imu_line = boost::regex(expr);
  boost::regex comment = boost::regex(RegexPattern::comment);

  while (std::getline(infile, line)) {
    if (line.empty()) {
      continue;
    } else if (boost::regex_match(line, match, comment)) {
      continue;
    } else if (boost::regex_match(line, match, imu_line)) {

//      int identifier = std::stoi(match[1]);

      int timestampS = std::stoi(match[2]);
      int timestampNS = std::stod(match[3]) * std::pow(10, 9 - match[3].length());

      float gx = std::stof(match[4]);
      float gy = std::stof(match[5]);
      float gz = std::stof(match[6]);
      float ax = std::stof(match[7]);
      float ay = std::stof(match[8]);
      float az = std::stof(match[9]);

      auto IMU_frame = new SLAMInMemoryFrame();
      IMU_frame->FrameSensor = IMU_sensor;
      IMU_frame->Timestamp.S = timestampS;
      IMU_frame->Timestamp.Ns = timestampNS;
      IMU_frame->Data = malloc(IMU_sensor->GetFrameSize(IMU_frame));

      ((float *)IMU_frame->Data)[0] = gx;
      ((float *)IMU_frame->Data)[1] = gy;
      ((float *)IMU_frame->Data)[2] = gz;

      ((float *)IMU_frame->Data)[3] = ax;
      ((float *)IMU_frame->Data)[4] = ay;
      ((float *)IMU_frame->Data)[5] = az;

      file.AddFrame(IMU_frame);

    } else {
      std::cerr << "Unknown line: " << line << std::endl;
      return false;
    }
  }
  return true;
}


SLAMFile *UZHFPVReader::GenerateSLAMFile() {

  std::string dirname = input;

  // check requirements for slamfile

  std::vector<std::string> requirements = {"img"};

  if (imu) {
    requirements.emplace_back("imu.txt");
  }

  if (gt) {
    requirements.emplace_back("groundtruth.txt");
  }

  if (stereo) {
    requirements.emplace_back("left_images.txt");
    requirements.emplace_back("right_images.txt");
  } else {
    requirements.emplace_back("images.txt");
  }

  if (events) {
    requirements.emplace_back("events.txt");
  }

  if (!checkRequirements(dirname, requirements)) {
    std::cerr << "Invalid folder." << std::endl;
    return nullptr;
  }

  // Setup slamfile

  auto slamfile_ptr = new SLAMFile();
  auto &slamfile = *slamfile_ptr;

  // load IMU data
  if (imu) {
    auto imu_sensor = new IMUSensor(dirname);

    // parameters from calibration imu.yaml
    imu_sensor->AcceleratorNoiseDensity = 0.1;
    imu_sensor->AcceleratorBiasDiffusion = 0.002;
    imu_sensor->GyroscopeNoiseDensity = 0.05;
    imu_sensor->GyroscopeBiasDiffusion = 4.0e-05;

    if (stereo) {
      // snapdragon
      imu_sensor->Rate = 500.0; // from calibration imu.yaml
    } else {
      // davis
      imu_sensor->Rate = 1000.0; // from calibration imu.yaml
    }

    imu_sensor->Index = slamfile.Sensors.size();
    slamfile.Sensors.AddSensor(imu_sensor);

    if(!loadUZHFPVIMUData(dirname, slamfile, imu_sensor)) {
      std::cerr << "Error while loading gt information." << std::endl;
      delete slamfile_ptr;
      return nullptr;
    }
  }

  // load GT
  if (gt) {
    auto gt_sensor = GTSensorBuilder()
        .name("GroundTruth")
        .description("GroundTruthSensor")
        .build();

    gt_sensor->Index = slamfile.Sensors.size();
    slamfile.Sensors.AddSensor(gt_sensor);

    if(!loadUZHFPVGroundTruthData(dirname, slamfile, gt_sensor)) {
      std::cerr << "Error while loading gt information." << std::endl;
      delete slamfile_ptr;
      return nullptr;
    }
  }

  return slamfile_ptr;
}

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
/*
bool loadIMUData(const std::string &dirname,
                 IMUSensor *IMU_sensor,
                 SLAMFile *file) {

  std::string line;

  boost::smatch match;
  std::ifstream infile(dirname + "/" + "data.csv");

  const std::string& ns = RegexPattern::nanoseconds;
  const std::string& num = RegexPattern::decimal;
  const std::string& start = RegexPattern::start;
  const std::string& end = RegexPattern::end;

  // format: timestamp,w_RS_S_x,w_RS_S_y,w_RS_S_z,a_RS_S_x,a_RS_S_y,a_RS_S_z
  std::string expr = start
                     + ns  + ","             // nanosecond timestamp
                     + num + ","             // w_RS_S_x
                     + num + ","             // w_RS_S_y
                     + num + ","             // w_RS_S_z
                     + num + ","             // a_RS_S_x
                     + num + ","             // a_RS_S_y
                     + num + "\\s*"          // a_RS_S_z
                     + end;

  boost::regex imu_line = boost::regex(expr);
  boost::regex comment = boost::regex(RegexPattern::comment);

  while (std::getline(infile, line)) {
    if (line.empty()) {
      continue;
    } else if (boost::regex_match(line, match, comment)) {
      continue;
    } else if (boost::regex_match(line, match, imu_line)) {

      uint64_t timestamp = strtol(std::string(match[1]).c_str(), nullptr, 10);

      int timestampS = timestamp / 1000000000;
      int timestampNS = timestamp % 1000000000;

      float gx = std::stof(match[2]);
      float gy = std::stof(match[3]);
      float gz = std::stof(match[4]);
      float ax = std::stof(match[5]);
      float ay = std::stof(match[6]);
      float az = std::stof(match[7]);

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

      file->AddFrame(IMU_frame);

    } else {
      std::cerr << "Unknown line:" << line << std::endl;
      return false;
    }
  }
  return true;
}

bool loadGTData(const std::string &dirname,
                Sensor *gt_sensor,
                SLAMFile *file) {

  std::string line;

  boost::smatch match;
  std::ifstream infile(dirname + "/" + "data.csv");

  const std::string& ns = RegexPattern::nanoseconds;
  const std::string& num = RegexPattern::decimal;
  const std::string& start = RegexPattern::start;
  const std::string& end = RegexPattern::end;

  // format: timestamp, p_RS_R_x, p_RS_R_y, p_RS_R_z, q_RS_w, q_RS_x, q_RS_y, q_RS_z, v_RS_R_x, v_RS_R_y, v_RS_R_z,
  //         b_w_RS_S_x, b_w_RS_S_y, b_w_RS_S_z, b_a_RS_S_x, b_a_RS_S_y, b_a_RS_S_z
  std::string expr = start
                     + ns  + ","             // nanosecond timestamp
                     + num + ","             // p_RS_R_x
                     + num + ","             // p_RS_R_y
                     + num + ","             // p_RS_R_z
                     + num + ","             // q_RS_w
                     + num + ","             // q_RS_x
                     + num + ","             // q_RS_y
                     + num + ","             // q_RS_z
                     + num + ","             // v_RS_R_x
                     + num + ","             // v_RS_R_y
                     + num + ","             // v_RS_R_z
                     + num + ","             // b_w_RS_S_x
                     + num + ","             // b_w_RS_S_y
                     + num + ","             // b_w_RS_S_z
                     + num + ","             // b_a_RS_S_x
                     + num + ","             // b_a_RS_S_y
                     + num + "\\s*"          // b_a_RS_S_z
                     + end;

  boost::regex groundtruth_line = boost::regex(expr);
  boost::regex comment = boost::regex(RegexPattern::comment);

  while (std::getline(infile, line)) {
    if (line.empty()) {
      continue;
    } else if (boost::regex_match(line, match, comment)) {
      continue;
    } else if (boost::regex_match(line, match, groundtruth_line)) {

      uint64_t timestamp = strtol(std::string(match[1]).c_str(), nullptr, 10);
      int timestampS = timestamp / 1000000000;
      int timestampNS = timestamp % 1000000000;

      float p_RS_R_x = std::stof(match[2]);  //  [m],
      float p_RS_R_y = std::stof(match[3]);  //  [m],
      float p_RS_R_z = std::stof(match[4]);  //  [m],

      float q_RS_w = std::stof(match[5]);  // [],
      float q_RS_x = std::stof(match[6]);  // [],
      float q_RS_y = std::stof(match[7]);  // [],
      float q_RS_z = std::stof(match[8]);  // [],

      // float v_RS_R_x  =  std::stof(match[9]) ; // [m s^-1]
      // float v_RS_R_y  =  std::stof(match[10]) ; // [m s^-1]
      // float v_RS_R_z  =  std::stof(match[11]) ; // [m s^-1]

      // float b_w_RS_S_x  =  std::stof(match[11]) ; // [rad s^-1],
      // float b_w_RS_S_y  =  std::stof(match[12]) ; // [rad s^-1],
      // float b_w_RS_S_z  =  std::stof(match[13]) ; // [rad s^-1],

      // float b_a_RS_S_x  =  std::stof(match[14]) ; // [m s^-2]
      // float b_a_RS_S_y  =  std::stof(match[15]) ; // [m s^-2]
      // float b_a_RS_S_z  =  std::stof(match[16]) ; // [m s^-2]

      Eigen::Matrix3f rotationMat = Eigen::Quaternionf(q_RS_w, q_RS_x, q_RS_y, q_RS_z).toRotationMatrix();
      Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
      pose.block(0, 0, 3, 3) = rotationMat;
      pose.block(0, 3, 3, 1) << p_RS_R_x, p_RS_R_y, p_RS_R_z;

      auto gt_frame = new SLAMInMemoryFrame();
      gt_frame->FrameSensor = gt_sensor;
      gt_frame->Timestamp.S = timestampS;
      gt_frame->Timestamp.Ns = timestampNS;
      gt_frame->Data = malloc(gt_sensor->GetFrameSize(gt_frame));

      memcpy(gt_frame->Data, pose.data(), gt_sensor->GetFrameSize(gt_frame));

      file->AddFrame(gt_frame);

    } else {
      std::cerr << "Unknown line:" << line << std::endl;
      return false;
    }
  }
  return true;
}
*/
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

  // load images

  return slamfile_ptr;
}

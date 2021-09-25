/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "include/EUROCMAV.h"
#include "include/utils/RegexPattern.h"
#include "include/utils/dataset_utils.h"
#include "io/sensor/sensor_builder.h"

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

#include <yaml-cpp/yaml.h>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

using namespace slambench::io;

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

SLAMFile *EUROCMAVReader::GenerateSLAMFile() {
  // first, see what sensors we have

  std::vector<std::string> sensor_directories;
  std::string input_dir = input;
  DIR *dir = opendir(input_dir.c_str());
  dirent *pdir;
  while ((pdir = readdir(dir)) != nullptr) {
    if (pdir->d_type == DT_DIR) {
      if (strcmp(pdir->d_name, ".") == 0) {
        continue;
      }
      if (strcmp(pdir->d_name, "..") == 0) {
        continue;
      }

      sensor_directories.emplace_back(pdir->d_name);
    }
  }

  auto slamfile = new SLAMFile();
  std::sort(sensor_directories.begin(), sensor_directories.end());
  for (auto &dirname : sensor_directories) {

    // try and get sensor.yaml file
    std::string cam_dirname = input_dir + "/" + dirname;
    std::string filename = cam_dirname + "/sensor.yaml";
    YAML::Node sensor = YAML::LoadFile(filename.c_str());

    // check sensor type
    auto sensor_type = sensor["sensor_type"].as<std::string>();

    if (sensor_type == "camera" and this->stereo) {
      std::cerr << "Found sensor type " << sensor_type << " from directory " << dirname << std::endl;

      // get pose
      Eigen::Matrix4f pose;

      for (int i = 0; i < 16; ++i) {
        int y = i % 4;
        int x = i / 4;
        std::cout << " " << sensor["T_BS"]["data"][i].as<float>();
        pose(x, y) = sensor["T_BS"]["data"][i].as<float>();
        if ((i + 1) % 4 == 0) std::cout << std::endl;
      }

      // get resolution
      int width = sensor["resolution"][0].as<int>();
      int height = sensor["resolution"][1].as<int>();
      float rate = sensor["rate_hz"].as<float>();

      // get intrinsics
      CameraSensor::intrinsics_t intrinsics = {
          sensor["intrinsics"][0].as<float>() / width,
          sensor["intrinsics"][1].as<float>() / height,
          sensor["intrinsics"][2].as<float>() / width,
          sensor["intrinsics"][3].as<float>() / height
      };

      // check expected distortion type
      if (sensor["distortion_model"].as<std::string>() != "radial-tangential") {
        std::cerr << "Unsupported distortion type for Eurocmav." << std::endl;
        exit(1);
      }

      // get distortion coefficients
      CameraSensor::distortion_coefficients_t distortion = {
          sensor["distortion_coefficients"][0].as<float>(),
          sensor["distortion_coefficients"][1].as<float>(),
          sensor["distortion_coefficients"][2].as<float>(),
          sensor["distortion_coefficients"][3].as<float>(),
          0
      };

      // Create a Grey sensor
      auto grey_sensor = GreySensorBuilder()
          .name(dirname)
          .rate(rate)
          .size(width, height)
          .description(sensor["comment"].as<std::string>())
          .intrinsics(intrinsics)
          .distortion(CameraSensor::distortion_type_t::RadialTangential, distortion)
          .pose(pose)
          .build();

      grey_sensor->Index = slamfile->Sensors.size();
      slamfile->Sensors.AddSensor(grey_sensor);

      // Create a RGB equivalent sensor
      auto rgb_sensor = RGBSensorBuilder()
          .name(dirname + "clone")
          .description("RGB clone from " + sensor["comment"].as<std::string>())
          .rate(rate)
          .size(width, height)
          .intrinsics(intrinsics)
          .pose(pose)
          .distortion(CameraSensor::distortion_type_t::RadialTangential, distortion)
          .build();

      rgb_sensor->Index = slamfile->Sensors.size();

      if (this->rgb) {
        slamfile->Sensors.AddSensor(rgb_sensor);
      }

      // now, load frames
      dir = opendir((cam_dirname + "/data/").c_str());
      while ((pdir = readdir(dir)) != nullptr) {
        if (pdir->d_type == DT_REG) {
          // Add the original Grey Image

          auto frame = new ImageFileFrame();
          frame->FrameSensor = grey_sensor;
          frame->filename = cam_dirname + "/data/" + pdir->d_name;

          uint64_t timestamp = strtol(pdir->d_name, nullptr, 10);
          frame->Timestamp.S = timestamp / 1000000000;
          frame->Timestamp.Ns = timestamp % 1000000000;

          slamfile->AddFrame(frame);

          if (this->rgb) {
            // Add the clone RGB
            auto rgb_frame = new ImageFileFrame();
            rgb_frame->FrameSensor = rgb_sensor;
            rgb_frame->filename = cam_dirname + "/data/" + pdir->d_name;
            rgb_frame->Timestamp.S = timestamp / 1000000000;
            rgb_frame->Timestamp.Ns = timestamp % 1000000000;
            slamfile->AddFrame(rgb_frame);
          }
        }
      }
    } else if (sensor_type == "imu" and this->imu) {
      std::cerr << "Found sensor type " << sensor_type << " from directory " << dirname << std::endl;

      auto imu_sensor = new IMUSensor(dirname);
      imu_sensor->Index = slamfile->Sensors.size();
      imu_sensor->Description = sensor["comment"].as<std::string>();
      imu_sensor->Rate = sensor["rate_hz"].as<float>();

      imu_sensor->GyroscopeNoiseDensity = sensor["gyroscope_noise_density"].as<float>();
      imu_sensor->GyroscopeDriftNoiseDensity = 4.0e-6;
      imu_sensor->GyroscopeBiasDiffusion = sensor["gyroscope_random_walk"].as<float>();
      imu_sensor->GyroscopeSaturation = 7.8;

      imu_sensor->AcceleratorNoiseDensity = sensor["accelerometer_noise_density"].as<float>();
      imu_sensor->AcceleratorDriftNoiseDensity = 4.0e-5;
      imu_sensor->AcceleratorBiasDiffusion = sensor["accelerometer_random_walk"].as<float>();
      imu_sensor->AcceleratorSaturation = 176.0;

//      double tau = 3600.0;  // # reversion time constant, currently not in use [s]
//      double g = 9.81007;   // # Earth's acceleration due to gravity [m/s^2]

//      Eigen::Vector3d a0 = {0.0, 0.0, 0.0};  // # Accelerometer bias [m/s^2]

      Eigen::Matrix4f pose;

      for (int i = 0; i < 16; ++i) {
        int y = i % 4;
        int x = i / 4;
        std::cout << " " << sensor["T_BS"]["data"][i].as<float>();
        pose(x, y) = sensor["T_BS"]["data"][i].as<float>();
        if ((i + 1) % 4 == 0) std::cout << std::endl;
      }

      imu_sensor->CopyPose(pose);

      slamfile->Sensors.AddSensor(imu_sensor);

      if (not loadIMUData(cam_dirname, imu_sensor, slamfile)) {
        delete slamfile;
        return nullptr;
      }
    } else if (sensor_type == "visual-inertial" and this->gt) {
      std::cerr << "Found sensor type " << sensor_type << " from directory " << dirname << std::endl;

      auto gt_sensor = GTSensorBuilder()
          .name(dirname)
          .description("Ground Truth")
          .build();

      gt_sensor->Index = slamfile->Sensors.size();
      slamfile->Sensors.AddSensor(gt_sensor);

      if (not loadGTData(cam_dirname, gt_sensor, slamfile)) {
        delete slamfile;
        return nullptr;
      }
    } else {
      std::cerr << "Unknown sensor type " << sensor_type << " from directory " << dirname << std::endl;
    }
  }

  return slamfile;
}

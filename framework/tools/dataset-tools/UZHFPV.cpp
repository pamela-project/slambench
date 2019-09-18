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
#include <io/Event.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/IMUSensor.h>
#include <io/sensor/Sensor.h>

#include <dirent.h>
#include <cstring>

#include <fstream>
#include <string>
#include <vector>

#include <iostream>

#include <io/sensor/EventCameraSensor.h>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

using namespace slambench::io;

bool loadUZHFPVGreyData(const std::string& dirname,
                        const std::string& filename,
                        SLAMFile &file,
                        CameraSensor* grey_sensor) {

  std::string line;

  std::ifstream infile(dirname + "/" + filename);

  boost::smatch match;

  boost::regex comment = boost::regex(RegexPattern::comment);

  const std::string& start = RegexPattern::start;
  const std::string& end = RegexPattern::end;
  const std::string& id = RegexPattern::decimal;
  const std::string& ts = RegexPattern::timestamp;
  const std::string& ws = RegexPattern::whitespace;
  const std::string& fn = RegexPattern::filename;

  // format: id timestamp filename
  std::string expr = start
      + id  + ws       // id
      + ts  + ws       // timestamp
      + fn  + ws       // filename
      + end;

  boost::regex grey_line = boost::regex(expr);

  while (std::getline(infile, line)) {
    if (line.empty()) {
      continue;
    } else if (boost::regex_match(line, match, comment)) {
      continue;
    } else if (boost::regex_match(line, match, grey_line)) {

//      int identifer = std::stoi(match[1]);

      int timestampS = std::stoi(match[2]);
      int timestampNS = std::stod(match[3]) * std::pow(10, 9 - match[3].length());
      std::string grey_filename = match[4];

      auto grey_frame = new ImageFileFrame();
      grey_frame->FrameSensor = grey_sensor;
      grey_frame->Timestamp.S = timestampS;
      grey_frame->Timestamp.Ns = timestampNS;

      std::stringstream frame_name;
      frame_name << dirname << "/" << grey_filename;
      grey_frame->Filename = frame_name.str();

      if (access(grey_frame->Filename.c_str(), F_OK) < 0) {
        printf("No Grey image for frame (%s)\n", frame_name.str().c_str());
        perror("");
        return false;
      }

      file.AddFrame(grey_frame);

    } else {
      std::cout << "Unknown line:" << line << std::endl;
      return false;
    }
  }
  return true;
}

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

bool loadUZHFPVEventData(const std::string &dirname,
                         SLAMFile &file,
                         EventCameraSensor *event_sensor) {

  using namespace slambench;

  std::string line;

  boost::smatch match;
  std::ifstream infile(dirname + "/" + "events.txt");

  const std::string& ts = RegexPattern::timestamp;
  const std::string& ws = RegexPattern::whitespace;
  const std::string& num = RegexPattern::integer;
  const std::string& start = RegexPattern::start;
  const std::string& end = RegexPattern::end;

  // format: timestamp x y polarity
  std::string expr = start
      + ts  + ws             // timestamp
      + num + ws             // x
      + num + ws             // y
      + num + end;           // polarity

  boost::regex events_line = boost::regex(expr);
  boost::regex comment = boost::regex(RegexPattern::comment);

  std::vector<Event> events = {};

  while (std::getline(infile, line)) {
    if (line.empty()) {
      continue;
    } else if (boost::regex_match(line, match, comment)) {
      continue;
    } else if (boost::regex_match(line, match, events_line)) {

      uint32_t timestampS = std::stoi(match[0]);
      uint32_t timestampNs = std::stod(match[2]) * std::pow(10, 9 - match[2].length());

      auto timestamp = TimeStamp{timestampS, timestampNs};
      int x = std::stoi(match[3]);
      int y = std::stoi(match[4]);
      bool p = std::stoi(match[5]);

      events.emplace_back(timestamp, x, y, p);

    } else {
      std::cerr << "Unknown line: " << line << std::endl;
      return false;
    }
  }

  size_t current_index = 0;
  auto current_ts = events[current_index].ts_;
  size_t i = current_index;

  // loop runs once per SLAM Frame
  while(current_index < events.size() - 1) {

    auto event_frame = new SLAMInMemoryFrame();
    event_frame->FrameSensor = event_sensor;
    event_frame->Timestamp = current_ts;

    // loop from current position until frame found with time difference greater than framerate
    for(i = current_index; i < events.size(); ++i) {
      auto delta = events[i].ts_ - current_ts;
      if (delta > std::chrono::milliseconds{20}) break;
    }

    size_t count = (i - 1) - current_index;
    size_t variable_size = sizeof(Event) * count;

    // copy from and to points into malloc'd memory
    event_frame->SetVariableSize(variable_size);
    event_frame->Data = malloc(variable_size);
    memcpy(event_frame->Data, &events[current_index], variable_size);

    file.AddFrame(event_frame);

    // set index to beginning of next frame
    current_index = i;
    current_ts = events[current_index].ts_;
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

  Sensor::pose_t pose = Eigen::Matrix4f::Identity();

  // load events data
  if (events) {
    auto event_sensor = new EventCameraSensor(dirname);
    event_sensor->Width = 346;
    event_sensor->Height = 260;
    event_sensor->Index = slamfile.Sensors.size();
    slamfile.Sensors.AddSensor(event_sensor);

    if (!loadUZHFPVEventData(dirname, slamfile, event_sensor)) {
      std::cerr << "Error while loading grey left stereo information." << std::endl;
      delete slamfile_ptr;
      return nullptr;
    }
  }

  // load camera data
  if (stereo) {

    // snapdragon stereo left
    auto left_sensor = GreySensorBuilder()
        .name("Grey Left Stereo")
        .size(640, 480)
        .pose(pose)
        .intrinsics(snapdragon_cam0_intrinsics)
        .distortion(CameraSensor::distortion_type_t::Equidistant, snapdragon_cam0_distortion)
        .rate(30)
        .build();

    left_sensor->Index = slamfile.Sensors.size();
    slamfile.Sensors.AddSensor(left_sensor);

    if (!loadUZHFPVGreyData(dirname, "left_images.txt", slamfile, left_sensor)) {
      std::cerr << "Error while loading grey left stereo information." << std::endl;
      delete slamfile_ptr;
      return nullptr;
    }

    // snapdragon stereo right
    auto right_sensor = GreySensorBuilder()
        .name("Grey Right Stereo")
        .size(640, 480)
        .pose(pose)
        .intrinsics(snapdragon_cam1_intrinsics)
        .distortion(CameraSensor::distortion_type_t::Equidistant, snapdragon_cam1_distortion)
        .rate(30) // from paper
        .build();

    right_sensor->Index = slamfile.Sensors.size();
    slamfile.Sensors.AddSensor(right_sensor);

    if (!loadUZHFPVGreyData(dirname, "right_images.txt", slamfile, right_sensor)) {
      std::cerr << "Error while loading grey right stereo information." << std::endl;
      delete slamfile_ptr;
      return nullptr;
    }

  } else {
    // davis grey camera
    auto grey_sensor = GreySensorBuilder()
        .name("Grey")
        .size(346, 260)
        .pose(pose)
        .intrinsics(davis_intrinsics)
        .distortion(CameraSensor::distortion_type_t::Equidistant, davis_distortion)
        .rate(50) // from paper
        .build();

    grey_sensor->Index = slamfile.Sensors.size();
    slamfile.Sensors.AddSensor(grey_sensor);

    if (!loadUZHFPVGreyData(dirname, "images.txt", slamfile, grey_sensor)) {
      std::cerr << "Error while loading Grey information." << std::endl;
      delete slamfile_ptr;
      return nullptr;
    }
  }

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

/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "include/SVO.h"
#include "include/utils/sensor_builder.h"

#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/Sensor.h>

#include <Eigen/Eigen>

#include <fstream>
#include <unistd.h>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

#include <boost/algorithm/string.hpp>

using namespace slambench::io;

constexpr CameraSensor::intrinsics_t SVOReader::svo_grey;

bool loadSVOGreyData(const std::string &dirname,
                     SLAMFile &file,
                     CameraSensor* grey_sensor) {

  for (int frame_no = 2; frame_no < 188; frame_no++) {
    auto grey_frame = new ImageFileFrame();
    grey_frame->FrameSensor = grey_sensor;
    grey_frame->Timestamp.S = frame_no - 2;

    std::stringstream frame_name;
    frame_name << dirname << "/img/frame_" << std::setw(6) << std::setfill('0') << frame_no << "_0.png";
    grey_frame->Filename = frame_name.str();

    if (access(grey_frame->Filename.c_str(), F_OK) < 0) {
      printf("No grey image for frame %d (%s)\n", frame_no, frame_name.str().c_str());
      return false;
    }

    file.AddFrame(grey_frame);
  }
  return true;
}

bool loadSVOGroundTruthData(const std::string &dirname, SLAMFile &file, GroundTruthSensor* gt_sensor) {

  std::string line;
  std::vector<std::string> results;
  std::ifstream infile(dirname + "/" + "trajectory_nominal.txt");

  int timestampS = 0;
  while (std::getline(infile, line)) {
    if (line.empty())
      continue;
    boost::split(results, line, boost::is_any_of(" "));
    // int timestampS = std::stoi(results[0]);
    float tx = std::stof(results[1]);
    float ty = std::stof(results[2]);
    float tz = std::stof(results[3]);

    float QX = std::stof(results[4]);
    float QY = std::stof(results[5]);
    float QZ = std::stof(results[6]);
    float QW = std::stof(results[7]);

    // the raw ground truth data are not normalized, we need normalized quaternions
    float Q_length = std::sqrt(std::pow(QX, 2) + std::pow(QY, 2) + std::pow(QZ, 2) + std::pow(QW, 2));

    QX /= Q_length;
    QY /= Q_length;
    QZ /= Q_length;
    QW /= Q_length;

    Eigen::Matrix3f rotationMat = Eigen::Quaternionf(QW, QX, QY, QZ).toRotationMatrix();
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

    pose.block(0, 0, 3, 3) = rotationMat;
    pose.block(0, 3, 3, 1) << tx, ty, tz;

    auto gt_frame = new SLAMInMemoryFrame();
    gt_frame->FrameSensor = gt_sensor;
    gt_frame->Timestamp.S = timestampS++;
    gt_frame->Data = malloc(gt_frame->GetSize());

    memcpy(gt_frame->Data, pose.data(), gt_frame->GetSize());

    file.AddFrame(gt_frame);
  }
  return true;
}

SLAMFile *SVOReader::GenerateSLAMFile() {
  std::string dirname = input;

  auto slamfile_ptr = new SLAMFile();
  auto &slamfile = *slamfile_ptr;

  Sensor::pose_t pose = Eigen::Matrix4f::Identity();
  Eigen::Matrix3f rotationMat = Eigen::Quaternionf(rotation[0],
                                                   rotation[1],
                                                   rotation[2],
                                                   rotation[3])
      .toRotationMatrix();

  pose.block(0, 0, 3, 3) = rotationMat;
  pose.block(0, 3, 3, 1) << translation[0], translation[1], translation[2];

  // load Grey
  auto grey_sensor = GreySensorBuilder()
      .name("Grey")
      .size(752, 480)
      .description("Grey")
      .pose(pose)
      .intrinsics(svo_grey)
      .build();

  grey_sensor->Index = slamfile.Sensors.size();
  slamfile.Sensors.AddSensor(grey_sensor);

  if (!loadSVOGreyData(dirname, slamfile, grey_sensor)) {
    std::cout << "Error while loading Grey information." << std::endl;
    delete slamfile_ptr;
    return nullptr;
  }

  // load GT
  auto gt_sensor = GTSensorBuilder()
      .name("GT")
      .description("GroundTruthSensor")
      .build();

  gt_sensor->Index = slamfile.Sensors.size();
  slamfile.Sensors.AddSensor(gt_sensor);

  if (!loadSVOGroundTruthData(dirname, slamfile, gt_sensor)) {
    std::cout << "Error while loading gt information." << std::endl;
    delete slamfile_ptr;
    return nullptr;
  }

  return slamfile_ptr;
}

/*

 Copyright (c) 2023 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

*/

#include "include/KITTI.h"
#include "include/utils/RegexPattern.h"
#include "include/utils/dataset_utils.h"
#include "io/sensor/sensor_builder.h"
#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/sensor/IMUSensor.h>
#include <io/sensor/LidarSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/format/PointCloud.h>
#include <Eigen/Eigen>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>


using namespace slambench::io;

bool loadKITTIRGBData(const std::string &dirname,
                      SLAMFile &file,
                      const Sensor::pose_t &pose,
                      const CameraSensor::intrinsics_t &intrinsics,
                      const CameraSensor::distortion_type_t &distortion_type,
                      const CameraSensor::distortion_coefficients_t &distortion,
                      const bool calib) {
    
    auto img_params = KITTIReader::get_image_params(calib);
    auto rgb_sensor = RGBSensorBuilder()
            .rate(img_params.rate)
            .size(img_params.width, img_params.height)
            .pose(pose)
            .intrinsics(intrinsics)
            .distortion(distortion_type, distortion)
            .index(file.Sensors.size())
            .build();
    
    file.Sensors.AddSensor(rgb_sensor);

    std::string line;
    std::ifstream infile(dirname + "/image_02/timestamps.txt");

    boost::smatch match;
    boost::regex comment = boost::regex(RegexPattern::comment);
    // format of timestamp: yyyy-mm-dd hr:min:sec.nsec 2011-09-30 12:40:59.442522880
    // extract hr, min, sec, nsec
    boost::regex pattern("(\\d+):(\\d+):(\\d+)\\.(\\d+)");

    int img_index = 0;
    while (std::getline(infile, line)) {
        if (line.empty()) {
            continue;
        } else if (boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, pattern)) {

            int hour = std::stoi(match[1]);
            int min = std::stoi(match[2]);
            int second = std::stoi(match[3]);
            int timestampS = hour * 3600 + min * 60 + second;
            int timestampNS = std::stoi(match[4]) * std::pow(10, 9 - match[4].length());

            auto rgb_frame = new ImageFileFrame();
            rgb_frame->FrameSensor = rgb_sensor;
            rgb_frame->Timestamp.S = timestampS;
            rgb_frame->Timestamp.Ns = timestampNS;

            // start from 0000000000.png
            std::stringstream tmp_filename;
            tmp_filename << std::setw(10) << std::setfill('0') << img_index;
            std::string rgb_filename = tmp_filename.str() + ".png";
            img_index++;

            std::stringstream frame_name;
            frame_name << dirname << "/image_02/data/" << rgb_filename;
            rgb_frame->filename = frame_name.str();

            if (access(rgb_frame->filename.c_str(), F_OK) < 0) {
                printf("No RGB image for frame (%s)\n", frame_name.str().c_str());
                perror("");
                return false;
            }

            file.AddFrame(rgb_frame);

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    return true;
}

bool loadKITTIGreyData(const std::string &dirname,
                      SLAMFile &file,
                      const Sensor::pose_t &pose,
                      const CameraSensor::intrinsics_t &intrinsics,
                      const CameraSensor::distortion_type_t &distortion_type,
                      const CameraSensor::distortion_coefficients_t &distortion,
                      const bool calib) {
    
    auto img_params = KITTIReader::get_image_params(calib);
    auto grey_sensor = GreySensorBuilder()
            .rate(img_params.rate)
            .size(img_params.width, img_params.height)
            .pose(pose)
            .intrinsics(intrinsics)
            .distortion(distortion_type, distortion)
            .index(file.Sensors.size())
            .build();
    
    file.Sensors.AddSensor(grey_sensor);

    std::string line;
    std::ifstream infile(dirname + "/image_00/timestamps.txt");

    boost::smatch match;
    boost::regex comment = boost::regex(RegexPattern::comment);
    // format of timestamp: yyyy-mm-dd hr:min:sec.nsec 2011-09-30 12:40:59.442522880
    // extract hr, min, sec, nsec
    boost::regex pattern("(\\d+):(\\d+):(\\d+)\\.(\\d+)");

    int img_index = 0;
    while (std::getline(infile, line)) {
        if (line.empty()) {
            continue;
        } else if (boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, pattern)) {

            int hour = std::stoi(match[1]);
            int min = std::stoi(match[2]);
            int second = std::stoi(match[3]);
            int timestampS = hour * 3600 + min * 60 + second;
            int timestampNS = std::stoi(match[4]) * std::pow(10, 9 - match[4].length());

            auto grey_frame = new ImageFileFrame();
            grey_frame->FrameSensor = grey_sensor;
            grey_frame->Timestamp.S = timestampS;
            grey_frame->Timestamp.Ns = timestampNS;

            // start from 0000000000.png
            std::stringstream tmp_filename;
            tmp_filename << std::setw(10) << std::setfill('0') << img_index;
            std::string grey_filename = tmp_filename.str() + ".png";
            img_index++;

            std::stringstream frame_name;
            frame_name << dirname << "/image_00/data/" << grey_filename;
            grey_frame->filename = frame_name.str();

            if (access(grey_frame->filename.c_str(), F_OK) < 0) {
                printf("No Grey image for frame (%s)\n", frame_name.str().c_str());
                perror("");
                return false;
            }

            file.AddFrame(grey_frame);

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    return true;
}

SLAMFile* KITTIReader::GenerateSLAMFile() {
    if(!(grey || rgb || lidar)) {
        std::cerr <<  "No sensors defined\n";
        return nullptr;
    }
    std::string dirname = input;
    std::vector<std::string> requirements = {};

    if (grey) {
        requirements.emplace_back("image_00/data");
        requirements.emplace_back("image_00/timestamps.txt");
    }

    if (rgb) {
        requirements.emplace_back("image_02/data");
        requirements.emplace_back("image_02/timestamps.txt");
    }

    if (lidar) {
        requirements.emplace_back("velodyne_points/data");
        requirements.emplace_back("velodyne_points/timestamps.txt");
        requirements.emplace_back("velodyne_points/timestamps_end.txt");
        requirements.emplace_back("velodyne_points/timestamps_start.txt");
    }

    if (imu) {
        requirements.emplace_back("oxts/data");
        requirements.emplace_back("oxts/dataformat.txt");
        requirements.emplace_back("oxts/timestamps.txt");
    }

    if (!checkRequirements(dirname, requirements)) {
        std::cerr << "Invalid folder." << std::endl;
        return nullptr;
    }

    auto slamfilep = new SLAMFile();
    SLAMFile &slamfile = *slamfilep;

    Sensor::pose_t pose = Eigen::Matrix4f::Identity();

    CameraSensor::intrinsics_t cam_intrinsics;

    CameraSensor::distortion_type_t cam_distortion_type;
    CameraSensor::distortion_coefficients_t cam_distortion;
    bool calib = true;

    KITTIReader::DatasetOrigin d_origin
        = get_sensor_params(cam_intrinsics, cam_distortion_type, cam_distortion);
    
    // Check the raw data type
    if (d_origin == KITTIReader::DatasetOrigin::RD11_09_30_CALIB) {

        std::cout << "using rectified parameter from 2011-09-30" << std::endl;
        calib = true;

    } else if (d_origin == KITTIReader::DatasetOrigin::RD11_10_03_CALIB) {

        std::cout << "implmentation incomplete for rectified 2011-10-03" << std::endl;
        return nullptr;

    } else if (d_origin == KITTIReader::DatasetOrigin::RD11_09_30) {

        std::cout << "implmentation incomplete for 2011-09-30" << std::endl;
        return nullptr;

    } else if (d_origin == KITTIReader::DatasetOrigin::RD11_10_03) {

        std::cout << "implmentation incomplete for 2011-10-03" << std::endl;
        return nullptr;

    } else {

        std::cout << "Invalid Path, please check d_origin at KITTI.cpp" << std::endl;
        return nullptr;

    }

    if (grey && !loadKITTIGreyData(dirname, slamfile, pose, cam_intrinsics, cam_distortion_type, cam_distortion, calib)) {
        std::cout << "Error while loading Grey information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    if (rgb && !loadKITTIRGBData(dirname, slamfile, pose, cam_intrinsics, cam_distortion_type, cam_distortion, calib)) {
        std::cout << "Error while loading RGB information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    return slamfilep;

}
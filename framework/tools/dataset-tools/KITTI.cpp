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
#include <boost/algorithm/string.hpp> 

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include "TimeStamp.h"


using namespace slambench::io;

std::list<slambench::TimeStamp> loadLeftRGBTimeStamps(const std::string &dirname) {

    std::string line;
    std::ifstream infile(dirname + "/image_02/timestamps.txt");
    std::list<slambench::TimeStamp> timestamps;

    boost::smatch match;
    boost::regex comment = boost::regex(RegexPattern::comment);
    // format of timestamp: yyyy-mm-dd hr:min:sec.nsec 2011-09-30 12:40:59.442522880
    // extract hr, min, sec, nsec
    boost::regex pattern("^\\d{4}-\\d{2}-\\d{2} (\\d{2}):(\\d{2}):(\\d{2})\\.(\\d{9})$");

    while (std::getline(infile, line)) {
        if (line.empty() || boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, pattern)) {

            int hour = std::stoi(match[1]);
            int min = std::stoi(match[2]);
            int second = std::stoi(match[3]);
            int timestampS = hour * 3600 + min * 60 + second;
            int timestampNS = std::stoi(match[4]) * std::pow(10, 9 - match[4].length());

            slambench::TimeStamp ts;
            ts.S = timestampS;
            ts.Ns = timestampNS;
            timestamps.push_back(ts);

        } else {
            std::cerr << "Unknown line in timestamps.txt of left RGB camera:" << line << std::endl;
            timestamps.clear();
            return timestamps;
        }
    }
    return timestamps;
}

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
    boost::regex pattern("^\\d{4}-\\d{2}-\\d{2} (\\d{2}):(\\d{2}):(\\d{2})\\.(\\d{9})$");

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
    boost::regex pattern("^\\d{4}-\\d{2}-\\d{2} (\\d{2}):(\\d{2}):(\\d{2})\\.(\\d{9})$");

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

bool loadKITTIGroundTruthData(const std::string &dirname, SLAMFile &file) {
    
    auto gt_sensor = GTSensorBuilder()
                .index(file.Sensors.size())
                .build();

    file.Sensors.AddSensor(gt_sensor);

    /***
     * No    Sequence Name       start   end
     * 00: 2011_10_03_drive_0027 000000 004540
     * 01: 2011_10_03_drive_0042 000000 001100
     * 02: 2011_10_03_drive_0034 000000 004660
     * 03: 2011_09_26_drive_0067 000000 000800
     * 04: 2011_09_30_drive_0016 000000 000270
     * 05: 2011_09_30_drive_0018 000000 002760
     * 06: 2011_09_30_drive_0020 000000 001100
     * 07: 2011_09_30_drive_0027 000000 001100
     * 08: 2011_09_30_drive_0028 001100 005170
     * 09: 2011_09_30_drive_0033 000000 001590
     * 10: 2011_09_30_drive_0034 000000 001200
    */
    std::ifstream infile;
    int start, end;
    if (dirname.find("2011_10_03_drive_0027") != std::string::npos) {
        // pose 00
        std::cerr << "Use pose 00 for 2011_10_03_drive_0027" << std::endl;
        infile.open(dirname+"/poses/00.txt");
        start = 0; end = 4540;

    } else if (dirname.find("2011_10_03_drive_0042") != std::string::npos) {
        // pose 01
        std::cerr << "Use pose 01 for 2011_10_03_drive_0042" << std::endl;
        infile.open(dirname+"/poses/01.txt");
        start = 0; end = 1100;

    } else if (dirname.find("2011_10_03_drive_0034") != std::string::npos) {
        // pose 02
        std::cerr << "Use pose 02 for 2011_10_03_drive_0034" << std::endl;
        infile.open(dirname+"/poses/02.txt");
        start = 0; end = 4660;

    } else if (dirname.find("2011_09_26_drive_0067") != std::string::npos) {
        // pose 03
        std::cerr << "Use pose 03 for 2011_09_26_drive_0067" << std::endl;
        infile.open(dirname+"/poses/03.txt");
        start = 0; end = 800;

    } else if (dirname.find("2011_09_30_drive_0016") != std::string::npos) {
        // pose 04
        std::cerr << "Use pose 04 for 2011_09_30_drive_0016" << std::endl;
        infile.open(dirname+"/poses/04.txt");
        start = 0; end = 270;

    } else if (dirname.find("2011_09_30_drive_0018") != std::string::npos) {
        // pose 05
        std::cerr << "Use pose 05 for 2011_09_30_drive_0018" << std::endl;
        infile.open(dirname+"/poses/05.txt");
        start = 0; end = 2760;

    } else if (dirname.find("2011_09_30_drive_0020") != std::string::npos) {
        // pose 06
        std::cerr << "Use pose 06 for 2011_09_30_drive_0020" << std::endl;
        infile.open(dirname+"/poses/06.txt");
        start = 0; end = 1100;

    } else if (dirname.find("2011_09_30_drive_0027") != std::string::npos) {
        // pose 07
        std::cerr << "Use pose 07 for 2011_09_30_drive_0027" << std::endl;
        infile.open(dirname+"/poses/07.txt");
        start = 0; end = 1100;

    } else if (dirname.find("2011_09_30_drive_0028") != std::string::npos) {
        // pose 08
        std::cerr << "Use pose 08 for 2011_09_30_drive_0028" << std::endl;
        infile.open(dirname+"/poses/08.txt");
        start = 1100; end = 5170;

    } else if (dirname.find("2011_09_30_drive_0033") != std::string::npos) {
        // pose 09
        std::cerr << "Use pose 09 for 2011_09_30_drive_0033" << std::endl;
        infile.open(dirname+"/poses/09.txt");
        start = 0; end = 1590;

    } else if (dirname.find("2011_09_30_drive_0034") != std::string::npos) {
        // pose 10
        std::cerr << "Use pose 10 for 2011_09_30_drive_0034" << std::endl;
        infile.open(dirname+"/poses/10.txt");
        start = 0; end = 1200;

    } else {
        std::cerr << "Invalid path to KITTI dataset groundtruth" << std::endl;
    }

    if (!infile.is_open()) {
        std::cerr << "Fail to open the pose file" << std::endl;
    }

    std::list<slambench::TimeStamp> timestamps = loadLeftRGBTimeStamps(dirname);
    if (timestamps.size() == 0) {
        std::cerr << "Unable to load timestamps of left RGB camera" << std::endl;
        return false;
    }

    std::string line;
    boost::smatch match;
    boost::regex comment = boost::regex(RegexPattern::comment);
    // pattern for 1.000000e+00
    boost::regex pattern("^[+-]?[0-9]+(\\.[0-9]+)?(?:e|E)[+-]?[0-9]+$");
    
    while (std::getline(infile, line)) {
        if (line.empty() || boost::regex_match(line, match, comment)) {
            continue;
        }

        std::vector<std::string> pose_values;
        std::vector<float> pose_elems;
        boost::split(pose_values, line, boost::is_any_of(" "));
        /***
         * pose value: r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
         * r11 r12 r13 tx
         * r21 r22 r23 ty
         * r31 r32 r33 tz
         * 0   0   0   1
         */
        
        for (const std::string& val : pose_values) {
            if (!boost::regex_match(val, pattern)) {
                std::cerr << val << " is NOT in the valid scientific notation format." << std::endl;
                return false;
            }
            float number = boost::lexical_cast<float>(val);
            pose_elems.push_back(number);
        }
        
        Eigen::Matrix4f pose;
        pose << pose_elems.at(0), pose_elems.at(1), pose_elems.at(2), pose_elems.at(3),
                pose_elems.at(4), pose_elems.at(5), pose_elems.at(6), pose_elems.at(7),
                pose_elems.at(8), pose_elems.at(9), pose_elems.at(10), pose_elems.at(11),
                0.0f,             0.0f,             0.0f,             1.0f;
        
        slambench::TimeStamp ts = timestamps.front();
        timestamps.pop_front();

        auto gt_frame = new SLAMInMemoryFrame();
        gt_frame->FrameSensor = gt_sensor;
        gt_frame->Timestamp.S = ts.S;
        gt_frame->Timestamp.Ns = ts.Ns;
        gt_frame->Data = malloc(gt_frame->GetSize());

        memcpy(gt_frame->Data, pose.data(), gt_frame->GetSize());

        file.AddFrame(gt_frame);
    }
    return true;
}

SLAMFile* KITTIReader::GenerateSLAMFile() {
    if(!(grey || rgb || lidar || gt)) {
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

    if (gt && !loadKITTIGroundTruthData(dirname, slamfile)) {
        std::cout << "Error while loading GroundTruth information." << std::endl;
        delete slamfilep;
        return nullptr;
    }
    
    return slamfilep;

}
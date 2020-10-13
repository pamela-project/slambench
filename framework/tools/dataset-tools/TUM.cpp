/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "include/TUM.h"
#include "include/utils/RegexPattern.h"
#include "include/utils/dataset_utils.h"
#include "io/sensor/sensor_builder.h"
#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/sensor/AccelerometerSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/PointCloudSensor.h>
#include <io/format/PointCloud.h>
#include <Eigen/Eigen>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include <iostream>
#include <fstream>


using namespace slambench::io;

bool loadTUMDepthData(const std::string &dirname,
                      SLAMFile &file,
                      const Sensor::pose_t &pose,
                      const DepthSensor::intrinsics_t &intrinsics,
                      const CameraSensor::distortion_coefficients_t &distortion,
                      const DepthSensor::disparity_params_t &disparity_params,
                      const DepthSensor::disparity_type_t &disparity_type,
                      const CameraSensor::distortion_type_t &distortion_type) {


    auto img_params = TUMReader::get_image_params();
    auto depth_sensor = DepthSensorBuilder()
            .name("Depth")
            .rate(img_params.rate)
            .size(img_params.width, img_params.height)
            .pose(pose)
            .intrinsics(intrinsics)
            .distortion(distortion_type, distortion)
            .disparity(disparity_type, disparity_params)
            .index(file.Sensors.size())
            .build();

    file.Sensors.AddSensor(depth_sensor);

    std::string line;

    std::ifstream infile(dirname + "/" + "depth.txt");

    boost::smatch match;

    boost::regex comment = boost::regex(RegexPattern::comment);

    const std::string& start = RegexPattern::start;
    const std::string& end = RegexPattern::end;
    const std::string& ts = RegexPattern::timestamp;
    const std::string& ws = RegexPattern::whitespace;
    const std::string& fn = RegexPattern::filename;

    // format: timestamp filename
    std::string expr = start
                       + ts  + ws       // timestamp
                       + fn + end;      // filename

    boost::regex depth_line = boost::regex(expr);

    while (std::getline(infile, line)) {
        if (line.empty()) {
            continue;
        } else if (boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, depth_line)) {
            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());
            std::string depth_filename = match[3];

            auto depth_frame = new ImageFileFrame();
            depth_frame->FrameSensor = depth_sensor;
            depth_frame->Timestamp.S = timestampS;
            depth_frame->Timestamp.Ns = timestampNS;

            std::stringstream frame_name;
            frame_name << dirname << "/" << depth_filename;
            depth_frame->filename = frame_name.str();

            if (access(depth_frame->filename.c_str(), F_OK) < 0) {
                printf("No depth image for frame (%s)\n", frame_name.str().c_str());
                perror("");
                return false;
            }

            file.AddFrame(depth_frame);

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    return true;
}


bool loadTUMRGBData(const std::string &dirname,
                    SLAMFile &file,
                    const Sensor::pose_t &pose,
                    const CameraSensor::intrinsics_t &intrinsics,
                    const CameraSensor::distortion_coefficients_t &distortion,
                    const CameraSensor::distortion_type_t &distortion_type) {
    auto img_params = TUMReader::get_image_params();
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

    std::ifstream infile(dirname + "/" + "rgb.txt");

    boost::smatch match;

    boost::regex comment = boost::regex(RegexPattern::comment);

    const std::string& start = RegexPattern::start;
    const std::string& end = RegexPattern::end;
    const std::string& ts = RegexPattern::timestamp;
    const std::string& ws = RegexPattern::whitespace;
    const std::string& fn = RegexPattern::filename;

    // format: timestamp filename
    std::string expr = start
                       + ts  + ws       // timestamp
                       + fn + end;      // filename

    boost::regex rgb_line = boost::regex(expr);

    while (std::getline(infile, line)) {
        if (line.empty()) {
            continue;
        } else if (boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, rgb_line)) {

            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());
            std::string rgb_filename = match[3];

            auto rgb_frame = new ImageFileFrame();
            rgb_frame->FrameSensor = rgb_sensor;
            rgb_frame->Timestamp.S = timestampS;
            rgb_frame->Timestamp.Ns = timestampNS;

            std::stringstream frame_name;
            frame_name << dirname << "/" << rgb_filename;
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

bool loadTUMGreyData(const std::string &dirname,
                     SLAMFile &file,
                     const Sensor::pose_t &pose,
                     const CameraSensor::intrinsics_t &intrinsics,
                     const CameraSensor::distortion_coefficients_t &distortion,
                     const CameraSensor::distortion_type_t &distortion_type) {
    auto img_params = TUMReader::get_image_params();
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

    std::ifstream infile(dirname + "/" + "rgb.txt");

    boost::smatch match;

    boost::regex comment = boost::regex(RegexPattern::comment);

    const std::string& start = RegexPattern::start;
    const std::string& end = RegexPattern::end;
    const std::string& ts = RegexPattern::timestamp;
    const std::string& ws = RegexPattern::whitespace;
    const std::string& fn = RegexPattern::filename;

    // format: timestamp filename
    std::string expr = start
                       + ts  + ws       // timestamp
                       + fn + end;      // filename

    boost::regex rgb_line = boost::regex(expr);

    while (std::getline(infile, line)) {
        if (line.empty()) {
            continue;
        } else if (boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, rgb_line)) {

            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());
            std::string rgb_filename = match[3];

            auto grey_frame = new ImageFileFrame();
            grey_frame->FrameSensor = grey_sensor;
            grey_frame->Timestamp.S = timestampS;
            grey_frame->Timestamp.Ns = timestampNS;

            std::stringstream frame_name;
            frame_name << dirname << "/" << rgb_filename;
            grey_frame->filename = frame_name.str();

            if (access(grey_frame->filename.c_str(), F_OK) < 0) {
                printf("No RGB image for frame (%s)\n", frame_name.str().c_str());
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


bool loadTUMGroundTruthData(const std::string &dirname , SLAMFile &file) {

    auto gt_sensor = GTSensorBuilder()
            .index(file.Sensors.size())
            .build();

    file.Sensors.AddSensor(gt_sensor);

    std::ifstream infile(dirname + "/" + "groundtruth.txt");

    std::string line;
    boost::smatch match;

    const std::string& ts = RegexPattern::timestamp;
    const std::string& ws = RegexPattern::whitespace;
    const std::string& dec = RegexPattern::decimal;
    const std::string& start = RegexPattern::start;
    const std::string& end = RegexPattern::end;

    // format: timestamp tx ty tz qx qy qz qw
    std::string expr = start
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
        if (line.empty() || boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, groundtruth_line)) {

            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());

            float tx = std::stof(match[3]);
            float ty = std::stof(match[4]);
            float tz = std::stof(match[5]);

            float QX = std::stof(match[6]);
            float QY = std::stof(match[7]);
            float QZ = std::stof(match[8]);
            float QW = std::stof(match[9]);

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

bool loadTUMAccelerometerData(const std::string &dirname, SLAMFile &file) {

    auto accelerometer_sensor = AccSensorBuilder()
            .index(file.Sensors.size())
            .build();

    file.Sensors.AddSensor(accelerometer_sensor);

    std::ifstream infile(dirname + "/" + "accelerometer.txt");

    std::string line;
    boost::smatch match;

    const std::string& ts = RegexPattern::timestamp;
    const std::string& ws = RegexPattern::whitespace;
    const std::string& dec = RegexPattern::decimal;
    const std::string& start = RegexPattern::start;
    const std::string& end = RegexPattern::end;

    // format: timestamp ax ay az
    std::string expr = start
                       + ts  + ws       // timestamp
                       + dec + ws       // ax
                       + dec + ws       // ay
                       + dec + end;     // az

    boost::regex accelerometer_line = boost::regex(expr);
    boost::regex comment = boost::regex(RegexPattern::comment);

    while (std::getline(infile, line)) {
        if (line.empty()) {
            continue;
        } else if (boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, accelerometer_line)) {

            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());
            float ax = std::stof(match[3]);
            float ay = std::stof(match[4]);
            float az = std::stof(match[5]);

            auto accelerometer_frame = new SLAMInMemoryFrame();
            accelerometer_frame->FrameSensor = accelerometer_sensor;
            accelerometer_frame->Timestamp.S = timestampS;
            accelerometer_frame->Timestamp.Ns = timestampNS;
            accelerometer_frame->Data = malloc(accelerometer_frame->GetSize());
            ((float *)accelerometer_frame->Data)[0] = ax;
            ((float *)accelerometer_frame->Data)[1] = ay;
            ((float *)accelerometer_frame->Data)[2] = az;

            file.AddFrame(accelerometer_frame);

        } else {
            std::cout << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    return true;
}

SLAMFile* TUMReader::GenerateSLAMFile () {

    if(!(grey || rgb || depth)) {
        std::cerr <<  "No sensors defined\n";
        return nullptr;
    }
    bool is_ethi = false;
    std::string dirname = input;

    const std::vector<std::string> requirements = {"accelerometer.txt",
                                                   "rgb.txt",
                                                   "rgb",
                                                   "depth.txt",
                                                   "depth",
                                                   "groundtruth.txt"};

    if (!checkRequirements(dirname, requirements)) {
        std::cerr << "Invalid folder." << std::endl;
        return nullptr;
    }


    auto slamfilep = new SLAMFile();
    SLAMFile & slamfile  = *slamfilep;

    Sensor::pose_t pose = Eigen::Matrix4f::Identity();

    /// Default parameters from freiburg1

    CameraSensor::intrinsics_t intrinsics_rgb;
    DepthSensor::intrinsics_t intrinsics_depth;

    CameraSensor::distortion_type_t distortion_type;
    CameraSensor::distortion_coefficients_t distortion_rgb;
    DepthSensor::distortion_coefficients_t distortion_depth;

    DepthSensor::disparity_params_t disparity_params;
    DepthSensor::disparity_type_t disparity_type;


    TUMReader::DatasetOrigin d_origin = get_sensor_params(disparity_params, disparity_type,
                                                        intrinsics_rgb, intrinsics_depth,
                                                        distortion_rgb, distortion_depth,
                                                        distortion_type);
    if (d_origin == TUMReader::DatasetOrigin::Default)
    {
        std::cout << "using default camera calibration parameters" << std::endl;
        std::cout << "warning: camera calibration might be wrong!" << std::endl;

    } else if (d_origin == TUMReader::DatasetOrigin::ETHI) {
        is_ethi = true;
        std::cout << "using ETH Illumination camera calibration parameters" << std::endl;

    } else {
        std::cout << "using freiburg" << d_origin << " camera calibration parameters" << std::endl;
    }

    if(gt && !loadTUMGroundTruthData(dirname, slamfile)) {
        std::cout << "Error while loading gt information." << std::endl;
        delete slamfilep;
        return nullptr;

    }

    if(accelerometer && !loadTUMAccelerometerData(dirname, slamfile)) {
        std::cout << "Error while loading Accelerometer information." << std::endl;
        delete slamfilep;
        return nullptr;

    }

    if (is_ethi)
        dirname = input + "/depth";
    if(depth && !loadTUMDepthData(dirname, slamfile,pose,intrinsics_depth,distortion_depth,disparity_params,disparity_type,distortion_type)) {
        std::cout << "Error while loading depth information." << std::endl;
        delete slamfilep;
        return nullptr;

    }

    if (is_ethi)
        dirname = input + "/rgb";
    if(grey && !loadTUMGreyData(dirname, slamfile,pose,intrinsics_rgb,distortion_rgb,distortion_type)) {
        std::cout << "Error while loading Grey information." << std::endl;
        delete slamfilep;
        return nullptr;

    }

    if(rgb && !loadTUMRGBData(dirname, slamfile,pose,intrinsics_rgb,distortion_rgb,distortion_type)) {
        std::cout << "Error while loading RGB information." << std::endl;
        delete slamfilep;
        return nullptr;

    }

    return slamfilep;
}




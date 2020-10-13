/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "include/BONN.h"
#include "include/utils/RegexPattern.h"
#include "include/utils/dataset_utils.h"
#include "io/sensor/sensor_builder.h"
#include "include/utils/PlyASCIIReader.h"

#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/format/PointCloud.h>
#include <io/sensor/GroundTruthSensor.h>
#include <Eigen/Eigen>

#include <iostream>

#include <io/sensor/PointCloudSensor.h>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

using namespace slambench::io;

constexpr CameraSensor::intrinsics_t BONNReader::intrinsics_rgb;
constexpr DepthSensor::intrinsics_t BONNReader::intrinsics_depth;
constexpr CameraSensor::distortion_coefficients_t BONNReader::distortion_rgb;
constexpr DepthSensor::distortion_coefficients_t BONNReader::distortion_depth;
constexpr DepthSensor::disparity_params_t BONNReader::disparity_params;
constexpr DepthSensor::disparity_type_t BONNReader::disparity_type;
constexpr CameraSensor::distortion_type_t BONNReader::distortion_type;

bool loadBONNDepthData(const std::string &dirname, SLAMFile &file) {

    std::string line;

    printf("Directory name: %s\n", dirname.c_str());
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

    int lineCount = 0;

    while (std::getline(infile, line)) {
        printf("line %d: %s\n", lineCount, line.c_str());
        lineCount++;

        if (line.empty()) {
            continue;
        } else if (boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, depth_line)) {

            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());

            std::string depth_filename = match[3];

            auto depth_frame = new ImageFileFrame();
            depth_frame->FrameSensor = *file.Sensors.end();
            depth_frame->Timestamp.S = timestampS;
            depth_frame->Timestamp.Ns = timestampNS;

            std::stringstream frame_name;
            frame_name << dirname << "/" << depth_filename;
            depth_frame->filename = frame_name.str();

            if (access(depth_frame->filename.c_str(), F_OK) < 0) {
                printf("No depth image for frame (%s)\n", frame_name.str().c_str());
                perror("");
                continue;
            }

            file.AddFrame(depth_frame);

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    return true;
}

bool loadBONNRGBData(const std::string &dirname,
                     SLAMFile &file) {

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
            rgb_frame->FrameSensor = *file.Sensors.end();
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

bool loadBONNGreyData(const std::string &dirname, SLAMFile &file) {

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
            grey_frame->FrameSensor = *file.Sensors.end();
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
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    return true;
}

bool loadBONNGroundTruthData(const std::string &dirname, SLAMFile &file) {

    std::ifstream infile(dirname + "/" + "groundtruth.txt");

    std::string line;
    boost::smatch match;

    const std::string& ts = RegexPattern::timestamp;
    const std::string& ws = RegexPattern::whitespace;
    const std::string& num = RegexPattern::number;
    const std::string& start = RegexPattern::start;
    const std::string& end = RegexPattern::end;

    // format: timestamp tx ty tz qx qy qz qw
    std::string expr = start
                       + ts  + ws       // timestamp
                       + num + ws       // tx
                       + num + ws       // ty
                       + num + ws       // tz
                       + num + ws       // qx
                       + num + ws       // qy
                       + num + ws       // qz
                       + num + end;     // qw

    boost::regex groundtruth_line = boost::regex(expr);
    boost::regex comment = boost::regex(RegexPattern::comment);

    while (std::getline(infile, line)) {
        if (line.empty()) {
            continue;
        } else if (boost::regex_match(line, match, comment)) {
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
            gt_frame->FrameSensor = *file.Sensors.end();
            gt_frame->Timestamp.S = timestampS;
            gt_frame->Timestamp.Ns = timestampNS;
            gt_frame->Data = malloc(gt_frame->GetSize());

            memcpy(gt_frame->Data, pose.data(), gt_frame->GetSize());

            file.AddFrame(gt_frame);
        } else {
            std::cerr << "Unknown line: " << line << std::endl;
            return false;
        }
    }
    return true;
}


bool loadBONNPointCloudData(slambench::io::SLAMFile &slamfile, const std::string& plyname) {
    auto pcd = new slambench::io::PointCloudSensor("PointCloud");
    pcd->Description = "Ground truth point cloud";
    pcd->Index = slamfile.Sensors.size();
    slamfile.Sensors.AddSensor(pcd);

    auto pointcloud = PlyASCIIReader::read(plyname);
    if(pointcloud == nullptr) {
        fprintf(stderr, "Could not read point cloud\n");
        return false;
    }

    auto rawpointcloud = pointcloud->ToRaw();

    auto pcloudframe = new SLAMInMemoryFrame();
    pcloudframe->FrameSensor = slamfile.GetSensor(PointCloudSensor::kPointCloudType);

    int numBytes = rawpointcloud.size();
    pcloudframe->Data = malloc(numBytes);
    pcloudframe->SetVariableSize(rawpointcloud.size());
    std::copy(rawpointcloud.data(),
              rawpointcloud.data() + numBytes,
              reinterpret_cast<char*>(pcloudframe->Data));

    slamfile.AddFrame(pcloudframe);

    return true;
}

SLAMFile *BONNReader::GenerateSLAMFile() {

    if (!(grey || rgb || depth)) {
        std::cerr << "No sensors defined\n";
        return nullptr;
    }

    std::string dirname = input;

    const std::vector<std::string> requirements = {"rgb.txt",
                                                   "rgb",
                                                   "depth.txt",
                                                   "depth",
                                                   "groundtruth.txt"};

    if (!checkRequirements(dirname, requirements)) {
        std::cerr << "Invalid folder." << std::endl;
        return nullptr;
    }

    auto slamfile_ptr = new SLAMFile();
    auto &slamfile = *slamfile_ptr;

    Sensor::pose_t pose = Eigen::Matrix4f::Identity();

    // load Depth
    if (depth) {
        auto depth_sensor = DepthSensorBuilder()
                .rate(image_params.rate)
                .size(image_params.width, image_params.height)
                .pose(pose)
                .intrinsics(intrinsics_depth)
                .distortion(distortion_type, distortion_depth)
                .disparity(disparity_type, disparity_params)
                .index(slamfile.Sensors.size())
                .build();

        slamfile.Sensors.AddSensor(depth_sensor);

        if (!loadBONNDepthData(dirname, slamfile)) {
            std::cout << "Error while loading depth information." << std::endl;
            delete slamfile_ptr;
            return nullptr;
        }
    }

    // load Grey
    if (grey) {
        auto grey_sensor = GreySensorBuilder()
                .rate(image_params.rate)
                .size(image_params.width, image_params.height)
                .pose(pose)
                .intrinsics(intrinsics_rgb)
                .distortion(distortion_type, distortion_rgb)
                .index(slamfile.Sensors.size())
                .build();

        slamfile.Sensors.AddSensor(grey_sensor);

        if (!loadBONNGreyData(dirname, slamfile)) {
            std::cerr << "Error while loading Grey information." << std::endl;
            delete slamfile_ptr;
            return nullptr;
        }
    }

    // load RGB
    if (rgb) {
        auto rgb_sensor = RGBSensorBuilder()
                .rate(image_params.rate)
                .size(image_params.width, image_params.height)
                .pose(pose)
                .intrinsics(intrinsics_rgb)
                .distortion(distortion_type, distortion_rgb)
                .index(slamfile.Sensors.size())
                .build();

        slamfile.Sensors.AddSensor(rgb_sensor);

        if (!loadBONNRGBData(dirname, slamfile)) {
            std::cerr << "Error while loading RGB information." << std::endl;
            delete slamfile_ptr;
            return nullptr;
        }
    }

    // load GT
    if (gt) {
        auto gt_sensor = GTSensorBuilder()
                .index(slamfile.Sensors.size())
                .build();

        slamfile.Sensors.AddSensor(gt_sensor);

        if(!loadBONNGroundTruthData(dirname, slamfile)) {
            std::cerr << "Error while loading gt information." << std::endl;
            delete slamfile_ptr;
            return nullptr;
        }
    }

    // load PointCloud
    if (!plyfile.empty() && !loadBONNPointCloudData(slamfile, plyfile)) {
        std::cerr << "Error while loading point cloud information." << std::endl;
        delete slamfile_ptr;
        return nullptr;
    }

    return slamfile_ptr;
}

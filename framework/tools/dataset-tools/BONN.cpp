/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of
 Manchester. Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "../dataset-tools/include/BONN.h"
#include "../dataset-tools/include/RegexPattern.h"

#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/format/PointCloud.h>
#include <io/sensor/GroundTruthSensor.h>
#include <Eigen/Eigen>

#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

using namespace slambench::io;

constexpr CameraSensor::intrinsics_t BONNReader::intrinsics_rgb;
constexpr DepthSensor::intrinsics_t BONNReader::intrinsics_depth;
constexpr CameraSensor::distortion_coefficients_t BONNReader::distortion_rgb;
constexpr DepthSensor::distortion_coefficients_t BONNReader::distortion_depth;

/*
 *
 * The dataset folder contains :
 * > depth  depth.txt  groundtruth.txt  rgb  rgb.txt
 *
 */

bool analyseBONNFolder(const std::string &dirname) {

    static const std::vector<std::string> requirements = {"rgb.txt",
                                                          "rgb",
                                                          "depth.txt",
                                                          "depth",
                                                          "groundtruth.txt"};

    try {
        if (!boost::filesystem::exists(dirname)) return false;

        boost::filesystem::directory_iterator end_itr;  // default construction yields past-the-end

        for (auto const &requirement : requirements) {
            bool seen = false;

            for (boost::filesystem::directory_iterator itr(dirname); itr != end_itr; ++itr) {
                if (requirement == itr->path().filename()) {
                    seen = true;
                }
            }

            if (!seen) {
                std::cout << "File not found: <dataset_dir>/" << requirement << std::endl;
                return false;
            }
        }
    } catch (boost::filesystem::filesystem_error &e) {
        std::cerr << "I/O Error with directory " << dirname << std::endl;
        std::cerr << e.what() << std::endl;
        return false;
    }

    return true;
}

bool loadBONNDepthData(const std::string &dirname,
                       SLAMFile &file,
                       const Sensor::pose_t &pose,
                       const DepthSensor::intrinsics_t &intrinsics,
                       const CameraSensor::distortion_coefficients_t &distortion,
                       const DepthSensor::disparity_params_t &disparity_params,
                       const DepthSensor::disparity_type_t &disparity_type) {

    DepthSensor *depth_sensor = new DepthSensor("Depth");
    depth_sensor->Index = 0;
    depth_sensor->Width = 640;
    depth_sensor->Height = 480;
    depth_sensor->FrameFormat = frameformat::Raster;
    depth_sensor->PixelFormat = pixelformat::D_I_16;
    depth_sensor->DisparityType = disparity_type;
    depth_sensor->Description = "Depth";
    depth_sensor->CopyPose(pose);
    depth_sensor->CopyIntrinsics(intrinsics);
    depth_sensor->CopyDisparityParams(disparity_params);
    depth_sensor->DistortionType = slambench::io::CameraSensor::RadialTangential;
    depth_sensor->CopyRadialTangentialDistortion(distortion);
    depth_sensor->Index = file.Sensors.size();
    depth_sensor->Rate = 30.0;

    file.Sensors.AddSensor(depth_sensor);

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

        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, depth_line)) {

            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());

            std::string depth_filename = match[3];

            ImageFileFrame *depth_frame = new ImageFileFrame();
            depth_frame->FrameSensor = depth_sensor;
            depth_frame->Timestamp.S = timestampS;
            depth_frame->Timestamp.Ns = timestampNS;

            std::stringstream frame_name;
            frame_name << dirname << "/" << depth_filename;
            depth_frame->Filename = frame_name.str();

            if (access(depth_frame->Filename.c_str(), F_OK) < 0) {
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
                     SLAMFile &file,
                     const Sensor::pose_t &pose,
                     const CameraSensor::intrinsics_t &intrinsics,
                     const CameraSensor::distortion_coefficients_t &distortion) {

    CameraSensor *rgb_sensor = new CameraSensor("RGB", CameraSensor::kCameraType);
    rgb_sensor->Index = 0;
    rgb_sensor->Width = 640;
    rgb_sensor->Height = 480;
    rgb_sensor->FrameFormat = frameformat::Raster;
    rgb_sensor->PixelFormat = pixelformat::RGB_III_888;
    rgb_sensor->Description = "RGB";
    rgb_sensor->CopyPose(pose);
    rgb_sensor->CopyIntrinsics(intrinsics);
    rgb_sensor->DistortionType = slambench::io::CameraSensor::RadialTangential;
    rgb_sensor->CopyRadialTangentialDistortion(distortion);
    rgb_sensor->Index = file.Sensors.size();
    rgb_sensor->Rate = 30.0;

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
        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, rgb_line)) {

            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());

            std::string rgb_filename = match[3];

            ImageFileFrame *rgb_frame = new ImageFileFrame();
            rgb_frame->FrameSensor = rgb_sensor;
            rgb_frame->Timestamp.S = timestampS;
            rgb_frame->Timestamp.Ns = timestampNS;

            std::stringstream frame_name;
            frame_name << dirname << "/" << rgb_filename;
            rgb_frame->Filename = frame_name.str();

            if (access(rgb_frame->Filename.c_str(), F_OK) < 0) {
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

bool loadBONNGreyData(const std::string &dirname,
                      SLAMFile &file,
                      const Sensor::pose_t &pose,
                      const CameraSensor::intrinsics_t &intrinsics,
                      const CameraSensor::distortion_coefficients_t &distortion) {

    CameraSensor *grey_sensor = new CameraSensor("Grey", CameraSensor::kCameraType);
    grey_sensor->Index = 0;
    grey_sensor->Width = 640;
    grey_sensor->Height = 480;
    grey_sensor->FrameFormat = frameformat::Raster;
    grey_sensor->PixelFormat = pixelformat::G_I_8;
    grey_sensor->Description = "Grey";

    grey_sensor->CopyPose(pose);
    grey_sensor->CopyIntrinsics(intrinsics);
    grey_sensor->CopyRadialTangentialDistortion(distortion);
    grey_sensor->DistortionType = slambench::io::CameraSensor::RadialTangential;
    grey_sensor->Index = file.Sensors.size();
    grey_sensor->Rate = 30.0;

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
        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, rgb_line)) {

            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());

            std::string rgb_filename = match[3];

            ImageFileFrame *grey_frame = new ImageFileFrame();
            grey_frame->FrameSensor = grey_sensor;
            grey_frame->Timestamp.S = timestampS;
            grey_frame->Timestamp.Ns = timestampNS;

            std::stringstream frame_name;
            frame_name << dirname << "/" << rgb_filename;
            grey_frame->Filename = frame_name.str();

            if (access(grey_frame->Filename.c_str(), F_OK) < 0) {
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

    GroundTruthSensor *gt_sensor = new GroundTruthSensor("GroundTruth");
    gt_sensor->Index = file.Sensors.size();
    gt_sensor->Description = "GroundTruthSensor";
    file.Sensors.AddSensor(gt_sensor);

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
        if (line.size() == 0) {
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

            SLAMInMemoryFrame *gt_frame = new SLAMInMemoryFrame();
            gt_frame->FrameSensor = gt_sensor;
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

SLAMFile *BONNReader::GenerateSLAMFile() {

    if (!(grey || rgb || depth)) {
        std::cerr << "No sensors defined\n";
        return nullptr;
    }

    std::string dirname = input;

    if (!analyseBONNFolder(dirname)) {
        std::cerr << "Invalid folder." << std::endl;
        return nullptr;
    }

    SLAMFile *slamfile_ptr = new SLAMFile();
    SLAMFile &slamfile = *slamfile_ptr;

    Sensor::pose_t pose = Eigen::Matrix4f::Identity();

    DepthSensor::disparity_params_t disparity_params = {0.001, 0.0};
    DepthSensor::disparity_type_t disparity_type = DepthSensor::affine_disparity;

    // load Depth
    if (depth && !loadBONNDepthData(dirname, slamfile, pose,
                                    BONNReader::intrinsics_depth, BONNReader::distortion_depth,
                                    disparity_params, disparity_type)) {
        std::cerr << "Error while loading depth information." << std::endl;
        delete slamfile_ptr;
        return nullptr;
    }

    // load Grey
    if (grey && !loadBONNGreyData(dirname, slamfile, pose,
                                  BONNReader::intrinsics_rgb, BONNReader::distortion_rgb)) {
        std::cerr << "Error while loading Grey information." << std::endl;
        delete slamfile_ptr;
        return nullptr;
    }

    // load RGB
    if (rgb && !loadBONNRGBData(dirname, slamfile, pose,
                                BONNReader::intrinsics_rgb, BONNReader::distortion_rgb)) {
        std::cerr << "Error while loading RGB information." << std::endl;
        delete slamfile_ptr;
        return nullptr;
    }

    // load GT
    if (gt && !loadBONNGroundTruthData(dirname, slamfile)) {
        std::cerr << "Error while loading gt information." << std::endl;
        delete slamfile_ptr;
        return nullptr;
    }

    return slamfile_ptr;
}

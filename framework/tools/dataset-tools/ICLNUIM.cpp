/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "include/ICLNUIM.h"
#include "include/utils/dataset_utils.h"
#include "io/sensor/sensor_builder.h"

#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/format/PointCloud.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/PointCloudSensor.h>
#include <Eigen/Eigen>

#include <boost/regex.hpp>

#include <utils/RegexPattern.h>
#include <fstream>
#include <iostream>
#include <fstream>
#include <Eigen/Core>

using namespace slambench::io;

constexpr DepthSensor::disparity_params_t ICLNUIMReader::disparity_params;
constexpr DepthSensor::disparity_type_t ICLNUIMReader::disparity_type;

struct float3 {
    float x, y, z;
};

void ICLNUIMReader::AddSensors(SLAMFile &file) {
    // TODO This information should come from the dataset !!
    Sensor::pose_t pose_depth = Eigen::Matrix4f::Identity();
    Sensor::pose_t pose = Eigen::Matrix4f::Identity();

    CameraSensor::intrinsics_t intrinsics{0.751875, -1.0, 0.4992185, 0.4989583};
    if (this->positive_focal) intrinsics[1] = 1.0;  // TODO : This is actually -1, bug .. no.

    CameraSensor::intrinsics_t intrinsics_depth{0.751875, -1.0, 0.4992185, 0.4989583};
    if (this->positive_focal) intrinsics_depth[1] = 1.0;  // TODO : This is actually -1, bug .. no.

    if (this->rgb) {
        this->rgb_sensor = RGBSensorBuilder()
                .rate(image_params.rate)
                .size(image_params.width, image_params.height)
                .pose(pose)
                .intrinsics(intrinsics)
                .index(file.Sensors.size())
                .build();

        file.Sensors.AddSensor(this->rgb_sensor);
    }

    if (this->depth) {
        this->depth_sensor = DepthSensorBuilder()
                .rate(image_params.rate)
                .size(image_params.width, image_params.height)
                .disparity(disparity_type, disparity_params)
                .pose(pose_depth)
                .intrinsics(intrinsics_depth)
                .index(file.Sensors.size())
                .build();

        file.Sensors.AddSensor(this->depth_sensor);
    }

    if (this->grey) {
        this->grey_sensor = GreySensorBuilder()
                .rate(image_params.rate)
                .size(image_params.width, image_params.height)
                .pose(pose)
                .intrinsics(intrinsics)
                .index(file.Sensors.size())
                .build();
        file.Sensors.AddSensor(this->grey_sensor);
    }

    if (this->gt) {
        this->gt_sensor = GTSensorBuilder()
                .rate(image_params.rate)
                .pose(pose)
                .build();

        this->gt_sensor->Index = file.Sensors.size();
        file.Sensors.AddSensor(this->gt_sensor);
    }
}

static void undistort_frame(slambench::io::SLAMFileFrame *frame, void *data) {

    auto depthMap = (uint16_t *)data;

    uint32_t w = dynamic_cast<CameraSensor*>(frame->FrameSensor)->Width;
    uint32_t h = dynamic_cast<CameraSensor*>(frame->FrameSensor)->Height;

    float u0 = 319.50;
    float v0 = 239.50;
    float fx = 481.20;
    float fy = -480.00;

    for (uint32_t v = 0; v < h; v++) {
        for (uint32_t u = 0; u < w; u++) {
            double u_u0_by_fx = (u - u0) / fx;
            double v_v0_by_fy = (v - v0) / fy;

            depthMap[u + v * w] = depthMap[u + v * w] / std::sqrt(u_u0_by_fx * u_u0_by_fx + v_v0_by_fy * v_v0_by_fy + 1);
        }
    }
}

static float3 normalise(const float3 &input) {
    float3 output = input;
    float magnitude = std::abs(std::sqrt((output.x * output.x) + (output.y * output.y) + (output.z * output.z)));
    if (output.x != 0) output.x /= magnitude;
    if (output.y != 0) output.y /= magnitude;
    if (output.z != 0) output.z /= magnitude;
    return output;
}

// Function to print matrix values existed in commit d2c324d and earlier
//void printMat(const Eigen::Matrix4f &mat, const std::string& color = "\033[0m");

bool FillPose(std::ifstream &istream, GroundTruthSensor::pose_t &pose, bool positive_focal) {

    const std::string& start = RegexPattern::start;
    const std::string& lkey = RegexPattern::lowercase_key;

    std::string key_expr = start + lkey;
    boost::regex key_regex(key_expr);

    const std::string& value = "([-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?)";
    std::string value_expr = value + ", " + value + ", " + value;
    boost::regex value_regex(value_expr);

    // find the important features for calculating the pose
    std::map<std::string, float3> kvs;
    std::string line;

    while(std::getline(istream, line)) {

        boost::cmatch match;
        if(!boost::regex_search(line.c_str(), match, key_regex)) {
            continue;
        }

        std::string key = match.str(0);

        if(!boost::regex_search(line.c_str(), match, value_regex)) {
            if (key != "cam_angle") {
                std::cout << "Error reading this line :" << line << std::endl;
                exit(1);
            }

            continue;
        }

        kvs[key].x = strtof(match.str(1).c_str(), nullptr);
        kvs[key].y = strtof(match.str(3).c_str(), nullptr);
        kvs[key].z = strtof(match.str(5).c_str(), nullptr);
    }

    // row 3
    float3 los = normalise(kvs["cam_dir"]);
    float3 up = normalise(kvs["cam_up"]);
    float3 right = normalise(kvs["cam_right"]);

    pose(0, 0) = right.x;
    pose(0, 1) = right.y;
    pose(0, 2) = right.z;
    pose(0, 3) = kvs["cam_pos"].x;
    pose(1, 0) = up.x;
    pose(1, 1) = up.y;
    pose(1, 2) = up.z;
    pose(1, 3) = kvs["cam_pos"].y;
    pose(2, 0) = los.x;
    pose(2, 1) = los.y;
    pose(2, 2) = los.z;
    pose(2, 3) = kvs["cam_pos"].z;
    pose(3, 0) = 0.0;
    pose(3, 1) = 0.0;
    pose(3, 2) = 0.0;
    pose(3, 3) = 1.0;

    if (positive_focal) {
        // First The ground truth is not align with the point cloud we need to flip
        pose(0, 3) *= -1.0;  // Pos

        // Second the ground truth is looking to the wrong Z direction
        static const GroundTruthSensor::pose_t origin = pose;
        pose = origin.inverse() * pose;
        pose(0, 2) *= -1.0;  // Rot
        pose(1, 2) *= -1.0;  // Rot
        pose(2, 0) *= -1.0;  // Rot
        pose(2, 1) *= -1.0;  // Rot
        pose = origin * pose;

        // Finally the camera is upside down !
        pose = origin.inverse() * pose;
        pose.block<2, 3>(0, 0) = -pose.block<2, 3>(0, 0);
        pose = origin * pose;
    }
    istream.close();
    return true;
}

bool GetPoseETHI(std::ifstream &istream, GroundTruthSensor::pose_t &pose) {

    std::string line;
    boost::smatch match;

    const std::string& f_no = RegexPattern::number;
    const std::string& ws = RegexPattern::whitespace;
    const std::string& dec = RegexPattern::decimal;
    const std::string& start = RegexPattern::start;
    const std::string& end = RegexPattern::end;

    // format: frame_no tx ty tz qx qy qz qw
    std::string expr = start
                       + f_no  + ws     // frame_no
                       + dec + ws       // tx
                       + dec + ws       // ty
                       + dec + ws       // tz
                       + dec + ws       // qx
                       + dec + ws       // qy
                       + dec + ws       // qz
                       + dec + end;     // qw

    boost::regex groundtruth_line = boost::regex(expr);
    boost::regex comment = boost::regex(RegexPattern::comment);

    std::getline(istream, line);
    if (line.empty()) {
        return false;
    } else if (boost::regex_match(line, match, comment)) {
        return false;
    } else if (boost::regex_match(line, match, groundtruth_line)) {

        float tx = std::stof(match[2]);
        float ty = std::stof(match[3]);
        float tz = std::stof(match[4]);

        float QX = std::stof(match[5]);
        float QY = std::stof(match[6]);
        float QZ = std::stof(match[7]);
        float QW = std::stof(match[8]);

        Eigen::Matrix3f rotationMat = Eigen::Quaternionf(QW, QX, QY, QZ).toRotationMatrix();
        pose = Eigen::Matrix4f::Identity();
        pose.block(0, 0, 3, 3) = rotationMat;
        pose.block(0, 3, 3, 1) << tx, ty, tz;

    } else {
        std::cout << "Unknown line:" << line << std::endl;
        return false;
    }
    return true;
}

bool ICLNUIMReader::GetFrame(const std::string &dirname, SLAMFile &file, int frame_no) {
    // two frames to add: one for rgb and one for depth

    const int frame_rate = 25;
    double frame_time = 1.0 / frame_rate;

    uint64_t total_ns = frame_time * frame_no * 1000000000;
    TimeStamp ts;
    ts.S = total_ns / 1000000000;
    ts.Ns = total_ns % 1000000000;

    if (rgb_sensor) {
        auto rgb_frame = new ImageFileFrame();
        rgb_frame->FrameSensor = rgb_sensor;

        rgb_frame->Timestamp = ts;

        std::stringstream frame_name;
        frame_name << dirname << "/scene_00_" << std::setw(4) << std::setfill('0') << frame_no << ".png";
        rgb_frame->filename = frame_name.str();

        if (access(rgb_frame->filename.c_str(), F_OK) < 0) {
            printf("No RGB image for frame %d (%s)\n", frame_no, frame_name.str().c_str());
            return false;
        }

        file.AddFrame(rgb_frame);
    }
    if (grey_sensor) {
        auto rgb_frame = new ImageFileFrame();
        rgb_frame->FrameSensor = grey_sensor;
        rgb_frame->Timestamp = ts;

        std::stringstream frame_name;
        frame_name << dirname << "/scene_00_" << std::setw(4) << std::setfill('0') << frame_no << ".png";
        rgb_frame->filename = frame_name.str();

        if (access(rgb_frame->filename.c_str(), F_OK) < 0) {
            printf("No Grey image for frame %d (%s)\n", frame_no, frame_name.str().c_str());
            perror("");
            return false;
        }

        file.AddFrame(rgb_frame);
    }

    if (depth_sensor) {
        SLAMFileFrame* depth_frame;
        std::stringstream frame_name;
        frame_name << dirname << "/scene_00_" << std::setw(4) << std::setfill('0') << frame_no << ".depth";

        // ETHI sequence
        if(istream.is_open()) {
            depth_frame = new ImageFileFrame();
            frame_name << ".png";
        }
        else {
            depth_frame = new TxtFileFrame();
            dynamic_cast<TxtFileFrame*>(depth_frame)->input_pixel_format = pixelformat::D_F_32;
            depth_frame->ProcessCallback = undistort_frame;
        }

        depth_frame->filename = frame_name.str();
        depth_frame->FrameSensor = depth_sensor;
        depth_frame->Timestamp = ts;

        if (access(depth_frame->filename.c_str(), F_OK) < 0) {
            printf("No depth image for frame %d (%s)\n", frame_no, frame_name.str().c_str());
            perror("");
            return false;
        }

        file.AddFrame(depth_frame);
    }

    if (gt_sensor) {
        auto frame = new SLAMInMemoryFrame();
        frame->FrameSensor = gt_sensor;
        frame->Timestamp = ts;
        frame->Data = malloc(frame->GetSize());
        bzero(frame->Data, frame->GetSize());

        if(istream.is_open()) {
            GetPoseETHI(istream, *((GroundTruthSensor::pose_t *)frame->Data));
        }
        else {
            std::stringstream frame_name;
            frame_name << dirname << "/scene_00_" << std::setw(4) << std::setfill('0') << frame_no << ".txt";

            if (access(frame_name.str().c_str(), F_OK) < 0) {
                printf("No metadata for frame %d (%s)\n", frame_no, frame_name.str().c_str());
                perror("");
                return false;
            }
            istream = std::ifstream(frame_name.str());
            FillPose(istream, *((GroundTruthSensor::pose_t *)frame->Data), this->positive_focal);
        }

        file.AddFrame(frame);
    }

    return true;
}

bool ICLNUIMReader::AddFrames(const std::string &dirname, SLAMFile &file) {
    int frame_no = 0;
    auto gt_file =  dirname+"/groundtruth.txt";
    if(access(gt_file.c_str(), F_OK) >= 0)
    {
        istream = std::ifstream(gt_file);
    }
    while (GetFrame(dirname, file, frame_no)) {
        frame_no++;
    }

    return true;
}

void AddPointCloudSensor(SLAMFile &slamfile) {
    auto pcd = new PointCloudSensor("PointCloud");
    pcd->Description = "Ground truth point cloud";
    pcd->Index = slamfile.Sensors.size();
    slamfile.Sensors.AddSensor(pcd);
}

void AddPlyFile(SLAMFile &slamfile, const std::string& plyname) {
    PlyReader plyreader;
    std::ifstream file(plyname.c_str());
    if (!file.good()) {
        fprintf(stderr, "Could not open PLY file\n");
        return;
    }
    auto pointcloud = plyreader.Read(file);
    if (pointcloud == nullptr) {
        fprintf(stderr, "Could not build point cloud\n");
        return;
    }
    auto rawpointcloud = pointcloud->ToRaw();

    auto pcloudframe = new SLAMInMemoryFrame();
    pcloudframe->FrameSensor = slamfile.GetSensor(PointCloudSensor::kPointCloudType);
    pcloudframe->Data = malloc(rawpointcloud.size());
    pcloudframe->SetVariableSize(rawpointcloud.size());
    memcpy(pcloudframe->Data, rawpointcloud.data(), rawpointcloud.size());
    slamfile.AddFrame(pcloudframe);
}

SLAMFile *ICLNUIMReader::GenerateSLAMFile() {
    std::cout << "Selection input file is " << this->input << std::endl;

    auto slamfile = new SLAMFile();

    if (!(grey || rgb || depth || gt)) {
        std::cout << "No sensor required." << std::endl;
        delete slamfile;
        return nullptr;
    }

    if (plyfile.length() != 0) {
        std::cout << "Add point cloud." << std::endl;
        AddPointCloudSensor(*slamfile);
    }

    AddSensors(*slamfile);

    if (plyfile.length() != 0) {
        AddPlyFile(*slamfile, plyfile);
    }

    if (!AddFrames(input, *slamfile)) {
        std::cout << "Failed to add frames." << std::endl;
        delete slamfile;
        return nullptr;
    }
    if(istream.is_open())
        istream.close();
    return slamfile;
}

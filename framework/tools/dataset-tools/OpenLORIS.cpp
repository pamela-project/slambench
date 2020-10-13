/*

 Copyright (c) 2019 Intel Corp.

 This code is licensed under the MIT License.

 */

#include "include/OpenLORIS.h"

#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/sensor/AccelerometerSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/GyroSensor.h>
#include <io/sensor/OdomSensor.h>
#include <Eigen/Eigen>

#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "io/sensor/sensor_builder.h"
#include "utils/dataset_utils.h"

using namespace slambench::io;

void Dijkstra(int n, int s, std::vector<std::vector<int> > G, std::vector<bool> &vis, std::vector<int> &d,
              std::vector<int> &pre) {
    fill(d.begin(), d.end(), INF);
    for (int i = 0; i < n; ++i) pre[i] = i;

    d[s] = 0;
    for (int i = 0; i < n; ++i) {
        int u = -1;
        int MIN = INF;
        for (int j = 0; j < n; ++j) {
            if (vis[j] == false && d[j] < MIN) {
                u = j;
                MIN = d[j];
            }
        }

        if (u == -1) return;
        vis[u] = true;
        for (int v = 0; v < n; ++v) {
            if (vis[v] == false && d[u] + G[u][v] < d[v]) {
                d[v] = d[u] + G[u][v];
                pre[v] = u;
            }
        }
    }
}

void DFSPrint(int s, int v, std::vector<int> pre, std::vector<int> &result) {
    if (v == s) {
        result.push_back(s);
        return;
    }
    DFSPrint(s, pre[v], pre, result);
    result.push_back(v);
}

Eigen::Matrix4f slambench::io::compute_trans_matrix(const std::string& input_name_1,
                                                    const std::string& input_name_2,
                                                    const std::string& filename) {
    auto yaml = YAML::LoadFile(filename);

    std::map<std::string, int> name_to_index;
    std::map<trans_direction, Eigen::Matrix4f> transforms;
    int num = 0;
    for (size_t i = 0; i < yaml["trans_matrix"].size(); i++) {
        int index_1, index_2;
        std::string name_1 = yaml["trans_matrix"][i]["parent_frame"].as<std::string>();
        if (name_to_index.count(name_1) == 0) {
            name_to_index[name_1] = num;
            num++;
        }
        std::string name_2 = yaml["trans_matrix"][i]["child_frame"].as<std::string>();
        if (name_to_index.count(name_2) == 0) {
            name_to_index[name_2] = num;
            num++;
        }
        index_1 = name_to_index[name_1];
        index_2 = name_to_index[name_2];
        trans_direction dir(index_1, index_2);
        Eigen::Matrix4f trans_matrix;
        trans_matrix << yaml["trans_matrix"][i]["matrix"]["data"][0].as<float>(),
                        yaml["trans_matrix"][i]["matrix"]["data"][1].as<float>(),
                        yaml["trans_matrix"][i]["matrix"]["data"][2].as<float>(),
                        yaml["trans_matrix"][i]["matrix"]["data"][3].as<float>(),
                        yaml["trans_matrix"][i]["matrix"]["data"][4].as<float>(),
                        yaml["trans_matrix"][i]["matrix"]["data"][5].as<float>(),
                        yaml["trans_matrix"][i]["matrix"]["data"][6].as<float>(),
                        yaml["trans_matrix"][i]["matrix"]["data"][7].as<float>(),
                        yaml["trans_matrix"][i]["matrix"]["data"][8].as<float>(),
                        yaml["trans_matrix"][i]["matrix"]["data"][9].as<float>(),
                        yaml["trans_matrix"][i]["matrix"]["data"][10].as<float>(),
                        yaml["trans_matrix"][i]["matrix"]["data"][11].as<float>(),
                        yaml["trans_matrix"][i]["matrix"]["data"][12].as<float>(),
                        yaml["trans_matrix"][i]["matrix"]["data"][13].as<float>(),
                        yaml["trans_matrix"][i]["matrix"]["data"][14].as<float>(),
                        yaml["trans_matrix"][i]["matrix"]["data"][15].as<float>();  // problem
        transforms[dir] = trans_matrix;
        trans_direction dir_inv(index_2, index_1);
        transforms[dir_inv] = trans_matrix.inverse();
    }

    int s = name_to_index[input_name_1];
    int v = name_to_index[input_name_2];
    int n = name_to_index.size();

    std::vector<std::vector<int> > G;
    std::vector<int> g;
    for (int j = 0; j < n; j++) {
        g.push_back(INF);
    }
    for (int i = 0; i < n; i++) {
        G.push_back(g);
    }
    for (auto& transform : transforms) {
        trans_direction index_pair = transform.first;
        G[index_pair.first][index_pair.second] = 1;
    }

    std::vector<bool> vis(n);
    std::vector<int> d(n);
    std::vector<int> pre(n);
    Dijkstra(n, s, G, vis, d, pre);
    std::vector<int> result;
    DFSPrint(s, v, pre, result);

    Eigen::Matrix4f result_matrix = Eigen::MatrixXf::Identity(4, 4);
    for (auto it = result.begin(); it != (result.end() - 1); it++) {
        trans_direction dir(*it, *(it + 1));
        Eigen::Matrix4f trans = transforms[dir];
        result_matrix = result_matrix * trans;
    }
    //    std::cout<<input_name_1<<" to "<<input_name_2<<std::endl;
    //    std::cout<<result_matrix<<std::endl;
    return result_matrix;
}

bool loadOpenLORISDepthData(const std::string &dirname,
                            const std::string &sensor_name,
                            SLAMFile &file,
                            const DepthSensor::disparity_params_t &disparity_params,
                            const DepthSensor::disparity_type_t &disparity_type,
                            const YAML::Node& yaml,
                            bool aligned_depth = false) {
    Eigen::Matrix4f pose =
            compute_trans_matrix("d400_color_optical_frame", sensor_name, dirname + "/trans_matrix.yaml");

    auto width = yaml[sensor_name]["width"].as<int>();
    auto height = yaml[sensor_name]["height"].as<int>();
    slambench::io::CameraSensor::intrinsics_t intrinsics;
    intrinsics[0] = yaml[sensor_name]["intrinsics"]["data"][0].as<float>() / width;
    intrinsics[1] = yaml[sensor_name]["intrinsics"]["data"][2].as<float>() / height;
    intrinsics[2] = yaml[sensor_name]["intrinsics"]["data"][1].as<float>() / width;
    intrinsics[3] = yaml[sensor_name]["intrinsics"]["data"][3].as<float>() / height;

    slambench::io::CameraSensor::distortion_coefficients_t distortion;
    slambench::io::CameraSensor::distortion_type_t distortion_type;
    if (yaml[sensor_name]["distortion_model"].as<std::string>() == "radial-tangential") {
        distortion_type = slambench::io::CameraSensor::RadialTangential;
        distortion[0] = yaml[sensor_name]["distortion_coefficients"]["data"][0].as<float>();
        distortion[1] = yaml[sensor_name]["distortion_coefficients"]["data"][1].as<float>();
        distortion[2] = yaml[sensor_name]["distortion_coefficients"]["data"][2].as<float>();
        distortion[3] = yaml[sensor_name]["distortion_coefficients"]["data"][3].as<float>();
        distortion[4] = 0;  //??
    }

    auto depth_sensor = DepthSensorBuilder()
            .name("Depth")
            .description("Depth")
            .rate(yaml[sensor_name]["fps"].as<float>())
            .size(width, height)
            .pose(pose)
            .intrinsics(intrinsics)
            .distortion(distortion_type, distortion)
            .disparity(disparity_type, disparity_params)
            .index(file.Sensors.size())
            .frameFormat(frameformat::Raster)
            .pixelFormat(pixelformat::D_I_16)
            .build();

    file.Sensors.AddSensor(depth_sensor);

    std::string line;
    std::string txt_name;
    if (aligned_depth) {
        txt_name = "aligned_depth.txt";
    } else {
        txt_name = "depth.txt";
    }
    std::ifstream infile(dirname + "/" + txt_name);
    boost::smatch match;

    while (std::getline(infile, line)) {
        if (line.empty()) {
            continue;
        } else if (boost::regex_match(line, match, boost::regex("^\\s*#.*$"))) {
            continue;
        } else if (boost::regex_match(line, match, boost::regex("^([0-9]+)[.]([0-9]+)\\s+(.*)$"))) {
            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());
            std::string depthfilename = match[3];

            auto depth_frame = new ImageFileFrame();
            depth_frame->FrameSensor = depth_sensor;
            depth_frame->Timestamp.S = timestampS;
            depth_frame->Timestamp.Ns = timestampNS;

            std::stringstream frame_name;
            frame_name << dirname << "/" << depthfilename;
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

bool loadOpenLORISImageData(const std::string &dirname,         // directory of data
                            const std::string &name_in_yaml,    // sensor name in sensors.yaml
                            const std::string &index_filename,  // filename of the data index (.txt)
                            bool greyscale,
                            SLAMFile &file,
                            const std::string &out_sensor_name, // name in .slam, should be identical
                            const YAML::Node yaml)
{
    slambench::io::CameraSensor::distortion_type_t distortion_type;
    if (yaml[name_in_yaml]["distortion_model"].as<std::string>() == "radial-tangential") {
        distortion_type = slambench::io::CameraSensor::RadialTangential;
    } else if (yaml[name_in_yaml]["distortion_model"].as<std::string>() == "Kannala-Brandt") {
        distortion_type = slambench::io::CameraSensor::KannalaBrandt;
    }
    auto pose = compute_trans_matrix("d400_color_optical_frame", name_in_yaml, dirname + "/trans_matrix.yaml");

    auto width = yaml[name_in_yaml]["width"].as<int>();
    auto height = yaml[name_in_yaml]["height"].as<int>();
    slambench::io::CameraSensor::intrinsics_t intrinsics;
    intrinsics[0] = yaml[name_in_yaml]["intrinsics"]["data"][0].as<float>() / width;
    intrinsics[1] = yaml[name_in_yaml]["intrinsics"]["data"][2].as<float>() / height;
    intrinsics[2] = yaml[name_in_yaml]["intrinsics"]["data"][1].as<float>() / width;
    intrinsics[3] = yaml[name_in_yaml]["intrinsics"]["data"][3].as<float>() / height;

    slambench::io::CameraSensor::distortion_coefficients_t distortion;
    distortion[0] = yaml[name_in_yaml]["distortion_coefficients"]["data"][0].as<float>();
    distortion[1] = yaml[name_in_yaml]["distortion_coefficients"]["data"][1].as<float>();
    distortion[2] = yaml[name_in_yaml]["distortion_coefficients"]["data"][2].as<float>();
    distortion[3] = yaml[name_in_yaml]["distortion_coefficients"]["data"][3].as<float>();
    distortion[4] = 0;

    auto sensor = CameraSensorBuilder()
            .name(out_sensor_name)
            .description(greyscale ? "Grey" : "RGB")
            .rate(yaml[name_in_yaml]["fps"].as<float>())
            .size(width, height)
            .pose(pose)
            .intrinsics(intrinsics)
            .distortion(distortion_type, distortion)
            .frameFormat(frameformat::Raster)
            .pixelFormat(greyscale ? pixelformat::G_I_8 : pixelformat::RGB_III_888)
            .index(file.Sensors.size())
            .build();

    file.Sensors.AddSensor(sensor);

    std::string line;
    std::ifstream infile(dirname + "/" + index_filename);
    boost::smatch match;

    while (std::getline(infile, line)) {
        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line, match, boost::regex("^\\s*#.*$"))) {
            continue;
        } else if (boost::regex_match(line, match, boost::regex("^([0-9]+)[.]([0-9]+)\\s+(.*)$"))) {
            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());
            std::string rgbfilename = match[3];

            auto grey_frame = new ImageFileFrame();
            grey_frame->FrameSensor = sensor;
            grey_frame->Timestamp.S = timestampS;
            grey_frame->Timestamp.Ns = timestampNS;

            std::stringstream frame_name;
            frame_name << dirname << "/" << rgbfilename;
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

bool loadOpenLORISGroundTruthData(const std::string &dirname, SLAMFile &file) {
    auto gt_sensor = GTSensorBuilder()
            .name("GroundTruth")
            .description("GroundTruthSensor")
            .index(file.Sensors.size())
            .build();
    file.Sensors.AddSensor(gt_sensor);
    // The target frame of gt in OpenLORIS is base_link. We change it here to d400_color, because most algorithms
    // would report estimated poses of d400_color (I guess). Though the transformation_ between the estimate target
    // frame and gt traget frame SHOULD be considered in traj alignment / evaluation / visualization, it is not.
    Eigen::Matrix4f trans_mat =
            compute_trans_matrix("d400_color_optical_frame", "base_link", dirname + "/trans_matrix.yaml");

    std::string line;
    boost::smatch match;
    std::ifstream infile(dirname + "/" + "groundtruth.txt");

    while (std::getline(infile, line)) {
        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line, match, boost::regex("^\\s*#.*$"))) {
            continue;
        } else if (boost::regex_match(
                line, match,
                boost::regex("^([0-9]+)[.]([0-9]+)\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s+([+-]?[0-9]"
                             "+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)"
                             "\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?"
                             ")[0-9]+)?)?)\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s+([+-]?[0-9]+(.[0-9]"
                             "+([Ee]([+-]?)[0-9]+)?)?)\\s*$"))) {
            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());

            float tx = std::stof(match[3]);
            float ty = std::stof(match[7]);
            float tz = std::stof(match[11]);
            float QX = std::stof(match[15]);
            float QY = std::stof(match[19]);
            float QZ = std::stof(match[23]);
            float QW = std::stof(match[27]);

            Eigen::Matrix3f rotationMat = Eigen::Quaternionf(QW, QX, QY, QZ).toRotationMatrix();
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            pose.block(0, 0, 3, 3) = rotationMat;
            pose.block(0, 3, 3, 1) << tx, ty, tz;
            pose = pose * trans_mat;

            auto gt_frame = new SLAMInMemoryFrame();
            gt_frame->FrameSensor = gt_sensor;
            gt_frame->Timestamp.S = timestampS;
            gt_frame->Timestamp.Ns = timestampNS;
            gt_frame->Data = malloc(gt_frame->GetSize());

            memcpy(gt_frame->Data, pose.data(), gt_frame->GetSize());

            file.AddFrame(gt_frame);

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    return true;
}

bool loadOpenLORISAccelerometerData(const std::string &dirname,
                                    const std::string &sensor_name,
                                    SLAMFile &file,
                                    const YAML::Node& yaml) {

    auto accelerometer_sensor = AccSensorBuilder()
            .name(sensor_name)
            .rate(yaml[sensor_name]["fps"].as<float>())
            .description("AccelerometerSensor")
            .index(file.Sensors.size())
            .pose(compute_trans_matrix("d400_color_optical_frame", sensor_name, dirname + "/trans_matrix.yaml"))
            .build();

    // accelerometer_sensor->AcceleratorNoiseDensity = 2.0000e-3;
    // accelerometer_sensor->AcceleratorDriftNoiseDensity = 4.0e-5;
    // accelerometer_sensor->AcceleratorBiasDiffusion = 3.0000e-3;
    // accelerometer_sensor->AcceleratorSaturation = 176.0;

    for (int i = 0; i < 12; i++) {
        accelerometer_sensor->Intrinsic[i] = yaml[sensor_name]["imu_intrinsic"]["data"][i].as<float>();
    }

    for (int i = 0; i < 3; i++) {
        accelerometer_sensor->NoiseVariances[i] = yaml[sensor_name]["noise_variances"]["data"][i].as<float>();
        accelerometer_sensor->BiasVariances[i] = yaml[sensor_name]["bias_variances"]["data"][i].as<float>();
    }

    file.Sensors.AddSensor(accelerometer_sensor);

    std::string line;
    boost::smatch match;
    std::ifstream infile(dirname + "/" + sensor_name + ".txt");

    while (std::getline(infile, line)) {
        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line, match, boost::regex("^\\s*#.*$"))) {
            continue;
        } else if (boost::regex_match(line, match,
                                      boost::regex("^([0-9]+)[.]([0-9]+)\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)"
                                                   "\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s+([+-]?[0-9]+(.["
                                                   "0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s*$"))) {
            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());

            auto accelerometer_frame = new SLAMInMemoryFrame();
            accelerometer_frame->FrameSensor = accelerometer_sensor;
            accelerometer_frame->Timestamp.S = timestampS;
            accelerometer_frame->Timestamp.Ns = timestampNS;
            accelerometer_frame->Data = malloc(accelerometer_frame->GetSize());
            ((float *)accelerometer_frame->Data)[0] = std::stof(match[3]); // ax
            ((float *)accelerometer_frame->Data)[1] = std::stof(match[7]); // ay
            ((float *)accelerometer_frame->Data)[2] = std::stof(match[11]); // az

            file.AddFrame(accelerometer_frame);

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    return true;
}

bool loadOpenLORISGyroData(const std::string &dirname,
                           const std::string &sensor_name,
                           SLAMFile &file,
                           const YAML::Node& yaml) {

    auto gyro_sensor = GyroSensorBuilder()
            .name(sensor_name)
            .rate(yaml[sensor_name]["fps"].as<float>())
            .description("GyroSensor")
            .index(file.Sensors.size())
            .pose(compute_trans_matrix("d400_color_optical_frame", sensor_name, dirname + "/trans_matrix.yaml"))
            .build();

    // gyro_sensor->GyroscopeNoiseDensity = 1.6968e-04;
    // gyro_sensor->GyroscopeDriftNoiseDensity = 4.0e-6;
    // gyro_sensor->GyroscopeBiasDiffusion = 1.9393e-05;
    // gyro_sensor->GyroscopeSaturation   =   7.8;

    for (int i = 0; i < 12; i++) {
        gyro_sensor->Intrinsic[i] = yaml[sensor_name]["imu_intrinsic"]["data"][i].as<float>();
    }
    for (int i = 0; i < 3; i++) {
        gyro_sensor->NoiseVariances[i] = yaml[sensor_name]["noise_variances"]["data"][i].as<float>();
        gyro_sensor->BiasVariances[i] = yaml[sensor_name]["bias_variances"]["data"][i].as<float>();
    }

    file.Sensors.AddSensor(gyro_sensor);

    std::string line;
    boost::smatch match;
    std::ifstream infile(dirname + "/" + sensor_name + ".txt");

    while (std::getline(infile, line)) {
        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line, match, boost::regex("^\\s*#.*$"))) {
            continue;
        } else if (boost::regex_match(line, match,
                                      boost::regex("^([0-9]+)[.]([0-9]+)\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)"
                                                   "\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s+([+-]?[0-9]+(.["
                                                   "0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s*$"))) {
            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());

            auto gyro_frame = new SLAMInMemoryFrame();
            gyro_frame->FrameSensor = gyro_sensor;
            gyro_frame->Timestamp.S = timestampS;
            gyro_frame->Timestamp.Ns = timestampNS;
            gyro_frame->Data = malloc(gyro_frame->GetSize());
            ((float *)gyro_frame->Data)[0] = std::stof(match[3]); // wx
            ((float *)gyro_frame->Data)[1] = std::stof(match[7]); // wy
            ((float *)gyro_frame->Data)[2] = std::stof( match[11]); // wz

            file.AddFrame(gyro_frame);

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    return true;
}

bool loadOpenLORISOdomData(const std::string &dirname, const std::string &sensor_name, SLAMFile &file) {

    auto odom_sensor = OdomSensorBuilder()
            .name(sensor_name)
            .description("OdomSensor")
            .index(file.Sensors.size())
            .pose(Eigen::Matrix4f::Identity())
            .build();
    file.Sensors.AddSensor(odom_sensor);

    std::string line;
    boost::smatch match;
    std::ifstream infile(dirname + "/" + "odom.txt");

    while (std::getline(infile, line)) {
        if (line.empty()) {
            continue;
        } else if (boost::regex_match(line, match, boost::regex("^\\s*#.*$"))) {
            continue;
        } else if (boost::regex_match(
                line, match,
                boost::regex("^([0-9]+)[.]([0-9]+)\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s+([+-]?[0-9]"
                             "+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)"
                             "\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?"
                             ")[0-9]+)?)?)\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s+([+-]?[0-9]+(.[0-9]"
                             "+([Ee]([+-]?)[0-9]+)?)?)\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s+([+-]?["
                             "0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?"
                             ")\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]"
                             "?)[0-9]+)?)?)\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s*$"))) {
            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());

            auto odom_frame = new SLAMInMemoryFrame();
            odom_frame->FrameSensor = odom_sensor;
            odom_frame->Timestamp.S = timestampS;
            odom_frame->Timestamp.Ns = timestampNS;
            odom_frame->Data = malloc(odom_frame->GetSize());
            ((float *)odom_frame->Data)[0] = std::stof(match[3]); // px
            ((float *)odom_frame->Data)[1] = std::stof(match[7]); // py
            ((float *)odom_frame->Data)[2] = std::stof(match[11]); // pz
            ((float *)odom_frame->Data)[3] = std::stof(match[15]); // ox
            ((float *)odom_frame->Data)[4] = std::stof(match[19]); // oy
            ((float *)odom_frame->Data)[5] = std::stof(match[23]); // oz
            ((float *)odom_frame->Data)[6] = std::stof(match[27]); // ow
            ((float *)odom_frame->Data)[7] = std::stof(match[31]); // lx
            ((float *)odom_frame->Data)[8] = std::stof(match[35]); // ly
            ((float *)odom_frame->Data)[9] = std::stof(match[39]); // lz
            ((float *)odom_frame->Data)[10] = std::stof(match[43]); // ax
            ((float *)odom_frame->Data)[11] = std::stof(match[47]); // ay
            ((float *)odom_frame->Data)[12] = std::stof(match[51]); // az

            file.AddFrame(odom_frame);

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    return true;
}

SLAMFile *OpenLORISReader::GenerateSLAMFile() {
    if (!(grey || color || depth || aligned_depth || fisheye1 || fisheye2)) {
        std::cerr << "No sensors defined\n";
        return nullptr;
    }

    std::string dirname = input;
    static const std::vector<std::string> requirements = {"d400_accelerometer.txt",
                                                          "d400_gyroscope.txt",
                                                          "t265_accelerometer.txt",
                                                          "t265_gyroscope.txt",
                                                          "odom.txt",
                                                          "color.txt",
                                                          "color",
                                                          "depth.txt",
                                                          "depth",
                                                          "aligned_depth.txt",
                                                          "aligned_depth",
                                                          "fisheye1.txt",
                                                          "fisheye1",
                                                          "fisheye2.txt",
                                                          "fisheye2",
                                                          "groundtruth.txt",
                                                          "sensors.yaml",
                                                          "trans_matrix.yaml"};
    if (!checkRequirements(dirname, requirements)) {
        std::cerr << "Invalid folder." << std::endl;
        return nullptr;
    }

    auto slamfilep = new SLAMFile();
    SLAMFile &slamfile = *slamfilep;

    DepthSensor::disparity_params_t disparity_params = {0.001, 0.0};
    DepthSensor::disparity_type_t disparity_type = DepthSensor::affine_disparity;

    auto yaml = YAML::LoadFile(dirname + "/sensors.yaml");

    /*** load RGB ***/
    if (color && !loadOpenLORISImageData(dirname, "d400_color_optical_frame", "color.txt", false, slamfile,"RGB", yaml)) {
        std::cout << "Error while loading OpenLORIS RGB information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    /*** load Grey ***/
    if (grey && !loadOpenLORISImageData(dirname, "d400_color_optical_frame", "color.txt", true, slamfile, "Grey", yaml)) {
        std::cout << "Error while loading OpenLORIS Grey information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    /*** load Aligned_Depth ***/
    if (aligned_depth &&
        !loadOpenLORISDepthData(dirname, "d400_color_optical_frame", slamfile, disparity_params, disparity_type,
                                yaml, true)) {
        std::cout << "Error while loading OpenLORIS depth information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    /*** load Depth ***/
    if (depth &&
        !loadOpenLORISDepthData(dirname, "d400_depth_optical_frame", slamfile, disparity_params, disparity_type, yaml)) {
        std::cout << "Error while loading OpenLORIS depth information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    /*** load Intel Realsense D400 Accelerometer ***/
    if (d400_accel && !loadOpenLORISAccelerometerData(dirname, "d400_accelerometer", slamfile, yaml)) {
        std::cout << "Error while loading Accelerometer information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    /*** load Intel Realsense D400 Gyro ***/
    if (d400_gyro && !loadOpenLORISGyroData(dirname, "d400_gyroscope",slamfile, yaml)) {
        std::cout << "Error while loading Gyro information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    /*** load Intel Realsense T265 Fisheye cameras ***/
    if (fisheye1 &&
        !loadOpenLORISImageData(dirname, "t265_fisheye1_optical_frame", "fisheye1.txt", true, slamfile,
                                "t265_fisheye1", yaml)) {
        std::cout << "Error while loading OpenLORIS fisheye1 information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    if (fisheye2 &&
        !loadOpenLORISImageData(dirname, "t265_fisheye2_optical_frame", "fisheye2.txt", true, slamfile,
                                "t265_fisheye2", yaml)) {
        std::cout << "Error while loading OpenLORIS fisheye2 information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    /*** load Intel Realsense T265 accelerometer ***/
    if (t265_accel && !loadOpenLORISAccelerometerData(dirname, "t265_accelerometer",  slamfile, yaml)) {
        std::cout << "Error while loading Accelerometer information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    /*** load Intel Realsense T265 accelerometer ***/
    if (t265_gyro && !loadOpenLORISGyroData(dirname, "t265_gyroscope",slamfile, yaml)) {
        std::cout << "Error while loading Gyro information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    /*** load wheel odometry ***/
    if (odom && !loadOpenLORISOdomData(dirname, "odometer", slamfile)) {
        std::cout << "Error while loading Odom information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    /*** load GT ***/
    if (gt && !loadOpenLORISGroundTruthData(dirname, slamfile)) {
        std::cout << "Error while loading gt information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    return slamfilep;
}

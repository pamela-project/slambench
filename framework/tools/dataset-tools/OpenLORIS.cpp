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

Eigen::Matrix4f slambench::io::compute_trans_matrix(std::string input_name_1, std::string input_name_2,
                                                    std::string filename) {
    YAML::Node f = YAML::LoadFile(filename.c_str());

    std::map<std::string, int> name_to_index;
    std::map<trans_direction, Eigen::Matrix4f> transations;
    int num = 0;
    for (size_t i = 0; i < f["trans_matrix"].size(); i++) {
        int index_1, index_2;
        std::string name_1 = f["trans_matrix"][i]["parent_frame"].as<std::string>();
        if (name_to_index.count(name_1) == 0) {
            name_to_index[name_1] = num;
            num++;
        }
        std::string name_2 = f["trans_matrix"][i]["child_frame"].as<std::string>();
        if (name_to_index.count(name_2) == 0) {
            name_to_index[name_2] = num;
            num++;
        }
        index_1 = name_to_index[name_1];
        index_2 = name_to_index[name_2];
        trans_direction dir(index_1, index_2);
        Eigen::Matrix4f trans_matrix;
        trans_matrix << f["trans_matrix"][i]["matrix"]["data"][0].as<float>(),
            f["trans_matrix"][i]["matrix"]["data"][1].as<float>(),
            f["trans_matrix"][i]["matrix"]["data"][2].as<float>(),
            f["trans_matrix"][i]["matrix"]["data"][3].as<float>(),
            f["trans_matrix"][i]["matrix"]["data"][4].as<float>(),
            f["trans_matrix"][i]["matrix"]["data"][5].as<float>(),
            f["trans_matrix"][i]["matrix"]["data"][6].as<float>(),
            f["trans_matrix"][i]["matrix"]["data"][7].as<float>(),
            f["trans_matrix"][i]["matrix"]["data"][8].as<float>(),
            f["trans_matrix"][i]["matrix"]["data"][9].as<float>(),
            f["trans_matrix"][i]["matrix"]["data"][10].as<float>(),
            f["trans_matrix"][i]["matrix"]["data"][11].as<float>(),
            f["trans_matrix"][i]["matrix"]["data"][12].as<float>(),
            f["trans_matrix"][i]["matrix"]["data"][13].as<float>(),
            f["trans_matrix"][i]["matrix"]["data"][14].as<float>(),
            f["trans_matrix"][i]["matrix"]["data"][15].as<float>();  // problem
        transations[dir] = trans_matrix;
        trans_direction dir_inv(index_2, index_1);
        transations[dir_inv] = trans_matrix.inverse();
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
    for (std::map<trans_direction, Eigen::Matrix4f>::iterator it = transations.begin(); it != transations.end(); ++it) {
        trans_direction index_pair = it->first;
        G[index_pair.first][index_pair.second] = 1;
    }

    std::vector<bool> vis(n);
    std::vector<int> d(n);
    std::vector<int> pre(n);
    Dijkstra(n, s, G, vis, d, pre);
    std::vector<int> result;
    DFSPrint(s, v, pre, result);

    Eigen::Matrix4f result_matrix = Eigen::MatrixXf::Identity(4, 4);
    for (std::vector<int>::iterator it = result.begin(); it != (result.end() - 1); it++) {
        trans_direction dir(*it, *(it + 1));
        Eigen::Matrix4f trans = transations[dir];
        result_matrix = result_matrix * trans;
    }
    //    std::cout<<input_name_1<<" to "<<input_name_2<<std::endl;
    //    std::cout<<result_matrix<<std::endl;
    return result_matrix;
}

bool analyseOpenLORISFolder(const std::string &dirname) {
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

    try {
        if (!boost::filesystem::exists(dirname)) return false;

        boost::filesystem::directory_iterator end_itr;  // default construction yields past-the-end
        for (auto requirement : requirements) {
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

bool loadOpenLORISDepthData(const std::string &dirname, const std::string &sensor_name, SLAMFile &file,
                            const DepthSensor::disparity_params_t &disparity_params,
                            const DepthSensor::disparity_type_t &disparity_type, bool aligned_depth = false) {
    std::string filename = dirname + "/sensors.yaml";
    YAML::Node f = YAML::LoadFile(filename.c_str());

    DepthSensor *depth_sensor = new DepthSensor(sensor_name);

    depth_sensor->Index = 0;
    depth_sensor->Width = f[sensor_name]["width"].as<int>();
    depth_sensor->Height = f[sensor_name]["height"].as<int>();
    depth_sensor->FrameFormat = frameformat::Raster;
    depth_sensor->PixelFormat = pixelformat::D_I_16;
    depth_sensor->DisparityType = disparity_type;
    depth_sensor->CopyDisparityParams(disparity_params);
    depth_sensor->Description = "Depth";
    Eigen::Matrix4f pose =
        compute_trans_matrix("d400_color_optical_frame", sensor_name, dirname + "/trans_matrix.yaml");
    depth_sensor->CopyPose(pose);

    depth_sensor->Intrinsics[0] = f[sensor_name]["intrinsics"]["data"][0].as<float>() / depth_sensor->Width;
    depth_sensor->Intrinsics[1] = f[sensor_name]["intrinsics"]["data"][2].as<float>() / depth_sensor->Height;
    depth_sensor->Intrinsics[2] = f[sensor_name]["intrinsics"]["data"][1].as<float>() / depth_sensor->Width;
    depth_sensor->Intrinsics[3] = f[sensor_name]["intrinsics"]["data"][3].as<float>() / depth_sensor->Height;

    if (f[sensor_name]["distortion_model"].as<std::string>() == "radial-tangential") {
        depth_sensor->DistortionType = slambench::io::CameraSensor::RadialTangential;
        depth_sensor->RadialTangentialDistortion[0] = f[sensor_name]["distortion_coefficients"]["data"][0].as<float>();
        depth_sensor->RadialTangentialDistortion[1] = f[sensor_name]["distortion_coefficients"]["data"][1].as<float>();
        depth_sensor->RadialTangentialDistortion[2] = f[sensor_name]["distortion_coefficients"]["data"][2].as<float>();
        depth_sensor->RadialTangentialDistortion[3] = f[sensor_name]["distortion_coefficients"]["data"][3].as<float>();
        depth_sensor->RadialTangentialDistortion[4] = 0;  //??
    }
    depth_sensor->Index = file.Sensors.size();
    depth_sensor->Rate = f[sensor_name]["fps"].as<float>();

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
        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line, match, boost::regex("^\\s*#.*$"))) {
            continue;
        } else if (boost::regex_match(line, match, boost::regex("^([0-9]+)[.]([0-9]+)\\s+(.*)$"))) {
            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());
            std::string depthfilename = match[3];

            ImageFileFrame *depth_frame = new ImageFileFrame();
            depth_frame->FrameSensor = depth_sensor;
            depth_frame->Timestamp.S = timestampS;
            depth_frame->Timestamp.Ns = timestampNS;

            std::stringstream frame_name;
            frame_name << dirname << "/" << depthfilename;
            depth_frame->Filename = frame_name.str();

            if (access(depth_frame->Filename.c_str(), F_OK) < 0) {
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
                            bool greyscale, SLAMFile &file,
                            const std::string &out_sensor_name)  // name in .slam, should be identical
{
    YAML::Node yaml = YAML::LoadFile((dirname + "/sensors.yaml").c_str());

    CameraSensor *sensor = new CameraSensor(out_sensor_name, CameraSensor::kCameraType);
    sensor->Index = 0;
    sensor->Width = yaml[name_in_yaml]["width"].as<int>();
    sensor->Height = yaml[name_in_yaml]["height"].as<int>();
    sensor->FrameFormat = frameformat::Raster;
    sensor->PixelFormat = greyscale ? pixelformat::G_I_8 : pixelformat::RGB_III_888;
    sensor->Description = greyscale ? "Grey" : "RGB";
    Eigen::Matrix4f pose =
        compute_trans_matrix("d400_color_optical_frame", name_in_yaml, dirname + "/trans_matrix.yaml");
    sensor->CopyPose(pose);

    sensor->Intrinsics[0] = yaml[name_in_yaml]["intrinsics"]["data"][0].as<float>() / sensor->Width;
    sensor->Intrinsics[1] = yaml[name_in_yaml]["intrinsics"]["data"][2].as<float>() / sensor->Height;
    sensor->Intrinsics[2] = yaml[name_in_yaml]["intrinsics"]["data"][1].as<float>() / sensor->Width;
    sensor->Intrinsics[3] = yaml[name_in_yaml]["intrinsics"]["data"][3].as<float>() / sensor->Height;

    if (yaml[name_in_yaml]["distortion_model"].as<std::string>() == "radial-tangential") {
        sensor->DistortionType = slambench::io::CameraSensor::RadialTangential;
    } else if (yaml[name_in_yaml]["distortion_model"].as<std::string>() == "Kannala-Brandt") {
        sensor->DistortionType = slambench::io::CameraSensor::KannalaBrandt;
    }

    sensor->RadialTangentialDistortion[0] = yaml[name_in_yaml]["distortion_coefficients"]["data"][0].as<float>();
    sensor->RadialTangentialDistortion[1] = yaml[name_in_yaml]["distortion_coefficients"]["data"][1].as<float>();
    sensor->RadialTangentialDistortion[2] = yaml[name_in_yaml]["distortion_coefficients"]["data"][2].as<float>();
    sensor->RadialTangentialDistortion[3] = yaml[name_in_yaml]["distortion_coefficients"]["data"][3].as<float>();
    sensor->RadialTangentialDistortion[4] = 0;
    sensor->Index = file.Sensors.size();
    sensor->Rate = yaml[name_in_yaml]["fps"].as<float>();

    file.Sensors.AddSensor(sensor);

    std::cout << "Grey camera sensor created..." << std::endl;

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

            ImageFileFrame *grey_frame = new ImageFileFrame();
            grey_frame->FrameSensor = sensor;
            grey_frame->Timestamp.S = timestampS;
            grey_frame->Timestamp.Ns = timestampNS;

            std::stringstream frame_name;
            frame_name << dirname << "/" << rgbfilename;
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

bool loadOpenLORISGroundTruthData(const std::string &dirname, SLAMFile &file) {
    GroundTruthSensor *gt_sensor = new GroundTruthSensor("GroundTruth");
    gt_sensor->Index = file.Sensors.size();
    gt_sensor->Description = "GroundTruthSensor";
    file.Sensors.AddSensor(gt_sensor);
    // The target frame of gt in OpenLORIS is base_link. We change it here to d400_color, because most algorithms
    // would report estimated poses of d400_color (I guess). Though the transformation between the estimate target
    // frame and gt traget frame SHOULD be considered in traj alignment / evaluation / visualization, it is not.
    Eigen::Matrix4f trans_mat =
        compute_trans_matrix("d400_color_optical_frame", "base_link", dirname + "/trans_matrix.yaml");

    if (!gt_sensor) {
        std::cout << "gt sensor not found..." << std::endl;
        return false;
    } else {
        std::cout << "gt sensor created..." << std::endl;
    }

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
            std::string temp3 = match[3], temp4 = match[7], temp5 = match[11], temp6 = match[15], temp7 = match[19],
                        temp8 = match[23], temp9 = match[27];
            char *p3 = (char *)temp3.data();
            char *p4 = (char *)temp4.data();
            char *p5 = (char *)temp5.data();
            char *p6 = (char *)temp6.data();
            char *p7 = (char *)temp7.data();
            char *p8 = (char *)temp8.data();
            char *p9 = (char *)temp9.data();

            float tx = std::atof(p3);
            float ty = std::atof(p4);
            float tz = std::atof(p5);
            float QX = std::atof(p6);
            float QY = std::atof(p7);
            float QZ = std::atof(p8);
            float QW = std::atof(p9);

            Eigen::Matrix3f rotationMat = Eigen::Quaternionf(QW, QX, QY, QZ).toRotationMatrix();
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            pose.block(0, 0, 3, 3) = rotationMat;

            pose.block(0, 3, 3, 1) << tx, ty, tz;

            pose = pose * trans_mat;

            SLAMInMemoryFrame *gt_frame = new SLAMInMemoryFrame();
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

bool loadOpenLORISAccelerometerData(const std::string &dirname, const std::string &sensor_name, SLAMFile &file) {
    std::string filename = dirname + "/sensors.yaml";
    YAML::Node f = YAML::LoadFile(filename.c_str());

    AccelerometerSensor *accelerometer_sensor = new AccelerometerSensor(sensor_name);
    accelerometer_sensor->Index = file.Sensors.size();
    accelerometer_sensor->Description = "AccelerometerSensor";

    accelerometer_sensor->Rate = f[sensor_name]["fps"].as<float>();

    // accelerometer_sensor->AcceleratorNoiseDensity = 2.0000e-3;
    // accelerometer_sensor->AcceleratorDriftNoiseDensity = 4.0e-5;
    // accelerometer_sensor->AcceleratorBiasDiffusion = 3.0000e-3;
    // accelerometer_sensor->AcceleratorSaturation = 176.0;

    for (int i = 0; i < 12; i++) {
        accelerometer_sensor->Intrinsic[i] = f[sensor_name]["imu_intrinsic"]["data"][i].as<float>();
    }

    for (int i = 0; i < 3; i++) {
        accelerometer_sensor->NoiseVariances[i] = f[sensor_name]["noise_variances"]["data"][i].as<float>();
        accelerometer_sensor->BiasVariances[i] = f[sensor_name]["bias_variances"]["data"][i].as<float>();
    }

    Eigen::Matrix4f pose =
        compute_trans_matrix("d400_color_optical_frame", sensor_name, dirname + "/trans_matrix.yaml");
    accelerometer_sensor->CopyPose(pose);

    file.Sensors.AddSensor(accelerometer_sensor);

    if (!accelerometer_sensor) {
        std::cout << "accelerometer_sensor not found..." << std::endl;
        return false;
    } else {
        std::cout << "accelerometer_sensor created..." << std::endl;
    }

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
            std::string temp3 = match[3], temp4 = match[7], temp5 = match[11];
            char *p3 = (char *)temp3.data();
            char *p4 = (char *)temp4.data();
            char *p5 = (char *)temp5.data();

            float ax = std::atof(p3);
            float ay = std::atof(p4);
            float az = std::atof(p5);

            SLAMInMemoryFrame *accelerometer_frame = new SLAMInMemoryFrame();
            accelerometer_frame->FrameSensor = accelerometer_sensor;
            accelerometer_frame->Timestamp.S = timestampS;
            accelerometer_frame->Timestamp.Ns = timestampNS;
            accelerometer_frame->Data = malloc(accelerometer_frame->GetSize());
            ((float *)accelerometer_frame->Data)[0] = ax;
            ((float *)accelerometer_frame->Data)[1] = ay;
            ((float *)accelerometer_frame->Data)[2] = az;

            file.AddFrame(accelerometer_frame);

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    return true;
}

bool loadOpenLORISGyroData(const std::string &dirname, const std::string &sensor_name, SLAMFile &file) {
    std::string filename = dirname + "/sensors.yaml";
    YAML::Node f = YAML::LoadFile(filename.c_str());

    GyroSensor *gyro_sensor = new GyroSensor(sensor_name);
    gyro_sensor->Index = file.Sensors.size();
    gyro_sensor->Description = "GyroSensor";

    gyro_sensor->Rate = f[sensor_name]["fps"].as<float>();

    // gyro_sensor->GyroscopeNoiseDensity = 1.6968e-04;
    // gyro_sensor->GyroscopeDriftNoiseDensity = 4.0e-6;
    // gyro_sensor->GyroscopeBiasDiffusion = 1.9393e-05;
    // gyro_sensor->GyroscopeSaturation   =   7.8;

    for (int i = 0; i < 12; i++) {
        gyro_sensor->Intrinsic[i] = f[sensor_name]["imu_intrinsic"]["data"][i].as<float>();
    }
    for (int i = 0; i < 3; i++) {
        gyro_sensor->NoiseVariances[i] = f[sensor_name]["noise_variances"]["data"][i].as<float>();
        gyro_sensor->BiasVariances[i] = f[sensor_name]["bias_variances"]["data"][i].as<float>();
    }

    Eigen::Matrix4f pose =
        compute_trans_matrix("d400_color_optical_frame", sensor_name, dirname + "/trans_matrix.yaml");
    gyro_sensor->CopyPose(pose);

    file.Sensors.AddSensor(gyro_sensor);

    if (!gyro_sensor) {
        std::cout << "gyro_sensor not found..." << std::endl;
        return false;
    } else {
        std::cout << "gyro_sensor created..." << std::endl;
    }

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

            std::string temp3 = match[3], temp4 = match[7], temp5 = match[11];
            char *p3 = (char *)temp3.data();
            char *p4 = (char *)temp4.data();
            char *p5 = (char *)temp5.data();

            float wx = std::atof(p3);
            float wy = std::atof(p4);
            float wz = std::atof(p5);

            SLAMInMemoryFrame *gyro_frame = new SLAMInMemoryFrame();
            gyro_frame->FrameSensor = gyro_sensor;
            gyro_frame->Timestamp.S = timestampS;
            gyro_frame->Timestamp.Ns = timestampNS;
            gyro_frame->Data = malloc(gyro_frame->GetSize());
            ((float *)gyro_frame->Data)[0] = wx;
            ((float *)gyro_frame->Data)[1] = wy;
            ((float *)gyro_frame->Data)[2] = wz;

            file.AddFrame(gyro_frame);

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    return true;
}

bool loadOpenLORISOdomData(const std::string &dirname, const std::string &sensor_name, SLAMFile &file) {
    OdomSensor *odom_sensor = new OdomSensor(sensor_name);
    odom_sensor->Index = file.Sensors.size();
    odom_sensor->Description = "OdomSensor";

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    odom_sensor->CopyPose(pose);

    file.Sensors.AddSensor(odom_sensor);

    if (!odom_sensor) {
        std::cout << "odom_sensor not found..." << std::endl;
        return false;
    } else {
        std::cout << "odom_sensor created..." << std::endl;
    }

    std::string line;

    boost::smatch match;
    std::ifstream infile(dirname + "/" + "odom.txt");

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
                                    "+([Ee]([+-]?)[0-9]+)?)?)\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s+([+-]?["
                                    "0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?"
                                    ")\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]"
                                    "?)[0-9]+)?)?)\\s+([+-]?[0-9]+(.[0-9]+([Ee]([+-]?)[0-9]+)?)?)\\s*$"))) {
            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) * std::pow(10, 9 - match[2].length());
            std::string temp3 = match[3], temp4 = match[7], temp5 = match[11], temp6 = match[15], temp7 = match[19],
                        temp8 = match[23], temp9 = match[27], temp10 = match[31], temp11 = match[35],
                        temp12 = match[39], temp13 = match[43], temp14 = match[47], temp15 = match[51];
            char *p3 = (char *)temp3.data();
            char *p4 = (char *)temp4.data();
            char *p5 = (char *)temp5.data();
            char *p6 = (char *)temp6.data();
            char *p7 = (char *)temp7.data();
            char *p8 = (char *)temp8.data();
            char *p9 = (char *)temp9.data();
            char *p10 = (char *)temp10.data();
            char *p11 = (char *)temp11.data();
            char *p12 = (char *)temp12.data();
            char *p13 = (char *)temp13.data();
            char *p14 = (char *)temp14.data();
            char *p15 = (char *)temp15.data();

            float px = std::atof(p3);
            float py = std::atof(p4);
            float pz = std::atof(p5);
            float ox = std::atof(p6);
            float oy = std::atof(p7);
            float oz = std::atof(p8);
            float ow = std::atof(p9);
            float lx = std::atof(p10);
            float ly = std::atof(p11);
            float lz = std::atof(p12);
            float ax = std::atof(p13);
            float ay = std::atof(p14);
            float az = std::atof(p15);

            SLAMInMemoryFrame *odom_frame = new SLAMInMemoryFrame();
            odom_frame->FrameSensor = odom_sensor;
            odom_frame->Timestamp.S = timestampS;
            odom_frame->Timestamp.Ns = timestampNS;
            odom_frame->Data = malloc(odom_frame->GetSize());
            ((float *)odom_frame->Data)[0] = px;
            ((float *)odom_frame->Data)[1] = py;
            ((float *)odom_frame->Data)[2] = pz;
            ((float *)odom_frame->Data)[3] = ox;
            ((float *)odom_frame->Data)[4] = oy;
            ((float *)odom_frame->Data)[5] = oz;
            ((float *)odom_frame->Data)[6] = ow;
            ((float *)odom_frame->Data)[7] = lx;
            ((float *)odom_frame->Data)[8] = ly;
            ((float *)odom_frame->Data)[9] = lz;
            ((float *)odom_frame->Data)[10] = ax;
            ((float *)odom_frame->Data)[11] = ay;
            ((float *)odom_frame->Data)[12] = az;

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

    if (!analyseOpenLORISFolder(dirname)) {
        std::cerr << "Invalid folder." << std::endl;
        return nullptr;
    }

    SLAMFile *slamfilep = new SLAMFile();
    SLAMFile &slamfile = *slamfilep;

    DepthSensor::disparity_params_t disparity_params = {0.001, 0.0};
    DepthSensor::disparity_type_t disparity_type = DepthSensor::affine_disparity;

    /**
     * load RGB
     */

    if (color && !loadOpenLORISImageData(dirname, "d400_color_optical_frame", "color.txt", false, slamfile, "RGB")) {
        std::cout << "Error while loading OpenLORIS RGB information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    /**
     * load Grey
     */

    if (grey && !loadOpenLORISImageData(dirname, "d400_color_optical_frame", "color.txt", true, slamfile, "Grey")) {
        std::cout << "Error while loading OpenLORIS Grey information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    /**
     * load Aligned_Depth
     */

    if (aligned_depth &&
        !loadOpenLORISDepthData(dirname, "d400_color_optical_frame", slamfile, disparity_params, disparity_type,
                                true)) {
        std::cout << "Error while loading OpenLORIS depth information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    /**
     * load Depth
     */

    if (depth &&
        !loadOpenLORISDepthData(dirname, "d400_depth_optical_frame", slamfile, disparity_params, disparity_type)) {
        std::cout << "Error while loading OpenLORIS depth information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    /**
     * load Accelerometer
     */
    if (d400_accel && !loadOpenLORISAccelerometerData(dirname, "d400_accelerometer", slamfile)) {
        std::cout << "Error while loading Accelerometer information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    if (d400_gyro && !loadOpenLORISGyroData(dirname, "d400_gyroscope", slamfile)) {
        std::cout << "Error while loading Gyro information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    /**
     * load Fisheyes
     */

    if (fisheye1 &&
        !loadOpenLORISImageData(dirname, "t265_fisheye1_optical_frame", "fisheye1.txt", true, slamfile,
                                "t265_fisheye1")) {
        std::cout << "Error while loading OpenLORIS fisheye1 information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    if (fisheye2 &&
        !loadOpenLORISImageData(dirname, "t265_fisheye2_optical_frame", "fisheye2.txt", true, slamfile,
                                "t265_fisheye2")) {
        std::cout << "Error while loading OpenLORIS fisheye2 information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    /**
     * load Accelerometer
     */
    if (t265_accel && !loadOpenLORISAccelerometerData(dirname, "t265_accelerometer", slamfile)) {
        std::cout << "Error while loading Accelerometer information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    if (t265_gyro && !loadOpenLORISGyroData(dirname, "t265_gyroscope", slamfile)) {
        std::cout << "Error while loading Gyro information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    if (odom && !loadOpenLORISOdomData(dirname, "odometer", slamfile)) {
        std::cout << "Error while loading Odom information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    /**
     * load GT
     */
    if (gt && !loadOpenLORISGroundTruthData(dirname, slamfile)) {
        std::cout << "Error while loading gt information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    return slamfilep;
}

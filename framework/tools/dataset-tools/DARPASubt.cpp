/*

 Copyright (c) 2023 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

*/

#include "include/DARPASubt.h"
#include "include/utils/RegexPattern.h"
#include "include/utils/dataset_utils.h"
#include "io/sensor/sensor_builder.h"
#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/sensor/LidarSensor.h>

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>
#include <boost/algorithm/string.hpp> 

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <cmath>
#include "TimeStamp.h"


using namespace slambench::io;

// You will need to resize image to be multiple of 16 if using darpra raw data
/*
bool resizeDARPASubtImage(const std::string &filename, int width, int height) {

    cv::Mat originalImage = cv::imread(filename);

    if (originalImage.empty()) {
        std::cout << "Could not read the image: " << filename << std::endl;
        return false;
    }

    if (originalImage.cols % 16 == 0 && originalImage.rows % 16 == 0) {
        return true;
    }

    cv::Mat resizedImage;
    cv::resize(originalImage, resizedImage, cv::Size(width, height));

    if (!cv::imwrite(filename, resizedImage)) {
        std::cout << "Could not save the resized image." << std::endl;
        return false;
    }

    return true;
}*/


bool loadDARPASubtGreyData(const std::string &dirname,
                      SLAMFile &file,
                      const float rate,
                      const int width,
                      const int height,
                      const Sensor::pose_t &pose,
                      const CameraSensor::intrinsics_t &intrinsics,
                      const CameraSensor::distortion_type_t &distortion_type,
                      const CameraSensor::distortion_coefficients_t &distortion) {
    
    auto grey_sensor = GreySensorBuilder()
            .rate(rate)
            .size(width, height)
            .pose(pose)
            .intrinsics(intrinsics)
            .distortion(distortion_type, distortion)
            .index(file.Sensors.size())
            .build();
    
    file.Sensors.AddSensor(grey_sensor);

    boost::regex image_pattern("(\\d+)\\.(\\d+).png");
    boost::smatch match;

    boost::filesystem::directory_iterator end_itr; // Default constructor yields past-the-end
    for (boost::filesystem::directory_iterator itr(dirname); itr != end_itr; ++itr) {

        if (boost::filesystem::is_regular_file(itr->status()) && itr->path().extension() == ".png") {

            std::string filename = itr->path().filename().string();

            if (boost::regex_match(filename, match, image_pattern)) {
                int timestampS = std::stoi(match[1].str());
                int timestampNS = std::stoi(match[2].str());

                auto grey_frame = new ImageFileFrame();
                grey_frame->FrameSensor = grey_sensor;
                grey_frame->Timestamp.S = timestampS;
                grey_frame->Timestamp.Ns = timestampNS;

                std::string grey_filename = itr->path().string();

                grey_frame->filename = grey_filename;

                if (access(grey_frame->filename.c_str(), F_OK) < 0) {
                    printf("No Grey image for frame (%s)\n", grey_filename.c_str());
                    perror("");
                    return false;
                }

                file.AddFrame(grey_frame);
            } else {
                std::cerr << "Unknown file:" << itr->path().filename() << std::endl;
                return false;
            }
        }
    }
    return true;
}


bool loadDARPASubtIMUData(const std::string &dirname, SLAMFile &file) {

    auto imu_sensor = new IMUSensor(dirname);
    imu_sensor->Index = file.Sensors.size();
    imu_sensor->Description = "IMU";
    imu_sensor->Rate = 10.0;

    imu_sensor->GyroscopeNoiseDensity = 0.000208;
    imu_sensor->GyroscopeBiasDiffusion = 4e-06;

    imu_sensor->AcceleratorNoiseDensity = 0.001249;
    imu_sensor->AcceleratorBiasDiffusion = 0.000106;

    Eigen::Matrix4f pose; // the location of imu origin at lidar coordinate
    pose << 0.99965732, -0.02617697,  0.0,  0.00432374,
            0.02617697,  0.99965732,  0.0,  0.037466,
            0.0,         0.0,         1.0,  0.063319,
            0.0,         0.0,         0.0,  1.0;

    imu_sensor->CopyPose(pose);

    file.Sensors.AddSensor(imu_sensor);
    std::cout << "IMU Sensor createrd..." << std::endl;
    std::cout << "HINT: the pose of IMU represents location of imu origin at lidar coordinate" << std::endl;

    std::string line;

    boost::smatch match;
    std::ifstream infile(dirname + "/" + "imu.csv");

    const std::string& num = RegexPattern::number;
    const std::string& start = RegexPattern::start;
    const std::string& end = RegexPattern::end;

    // new format: #counter, sec, nansec, wx, wy, wz, ax, ay, az
    std::string expr = start
        + num + ","             // counter
        + num + ","             // sec
        + num + ","             // nsec
        + num + ","             // wx
        + num + ","             // wy
        + num + ","             // wz
        + num + ","             // ax
        + num + ","             // ay
        + num + "\\s*"          // az
        + end;

    boost::regex imu_line = boost::regex(expr);
    boost::regex comment = boost::regex(RegexPattern::comment);

    
    bool is_first_pose = true;
    while (std::getline(infile, line)) {
        if (line.empty()) {
            continue;
        } else if (boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, imu_line)) {

            int timestampS = std::stoi(match[2]);
            int timestampNS = std::stoi(match[3]);

            float wx = std::stof(match[4]);
            float wy = std::stof(match[5]);
            float wz = std::stof(match[6]);
            float ax = std::stof(match[7]);
            float ay = std::stof(match[8]);
            float az = std::stof(match[9]);

            auto IMU_frame = new SLAMInMemoryFrame();
            IMU_frame->FrameSensor = imu_sensor;
            IMU_frame->Timestamp.S = timestampS;
            IMU_frame->Timestamp.Ns = timestampNS;
            IMU_frame->Data = malloc(imu_sensor->GetFrameSize(IMU_frame));

            ((float *)IMU_frame->Data)[0] = wx;
            ((float *)IMU_frame->Data)[1] = wy;
            ((float *)IMU_frame->Data)[2] = wz;

            ((float *)IMU_frame->Data)[3] = ax;
            ((float *)IMU_frame->Data)[4] = ay;
            ((float *)IMU_frame->Data)[5] = az;

            file.AddFrame(IMU_frame);

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    return true;
}


bool loadDARPASubtLidarData(const std::string &dirname, SLAMFile &file) {
    
    std::string lidar_dirname = dirname + "/velodyne_pcd";
    auto lidar_sensor = new LidarSensor(lidar_dirname);
    lidar_sensor->Index = file.Sensors.size();
    lidar_sensor->Rate = 10.0;
    lidar_sensor->BeamNum = 16;

    file.Sensors.AddSensor(lidar_sensor);
    std::cout << "Lidar sensor created... (It could take a while)" << std::endl;

    // convert to be same as KITTI set up
    Eigen::Matrix4f align_mat = (Eigen::Matrix4f() << 0.0, -1.0,  0.0,  0.0,
                                                      1.0,  0.0,  0.0,  0.0,
                                                      0.0,  0.0,  1.0,  0.0,
                                                      0.0,  0.0,  0.0,  1.0).finished();

    std::ifstream infile(lidar_dirname + "/timestamp.txt");
    if (infile.is_open()) { // If timestamp file exists
        // TODO: if there is a timestamp file
        infile.close();

    } else { // If timestamp file does not exist, infer from .pcd filenames

        boost::filesystem::directory_iterator end_iter;
        boost::regex pattern("(\\d+)\\.(\\d+).pcd");
        boost::smatch match;

        for (boost::filesystem::directory_iterator dir_iter(lidar_dirname); dir_iter != end_iter; ++dir_iter) {
            
            if (boost::filesystem::is_regular_file(dir_iter->status()) && dir_iter->path().extension() == ".pcd") {

                std::string filename = dir_iter->path().filename().string();

                if (boost::regex_match(filename, match, pattern)) {

                    // Extract timestamp from the filename
                    int timestampS = std::stoi(match[1].str());
                    int timestampNS = std::stoi(match[2].str());;

                    // ====== Load pointcloud for each frame
                    std::string lidar_file_pcd = lidar_dirname + "/" + filename;

                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

                    // Read the PCD file into the point cloud
                    if (pcl::io::loadPCDFile<pcl::PointXYZI>(lidar_file_pcd.c_str(), *cloud) == -1) {
                        std::cerr << "Failed to read file: " << lidar_file_pcd.c_str() << std::endl;
                        return false;
                    }

                    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
                    pcl::transformPointCloud(*cloud, *transformed_cloud, align_mat);

                    // Convert the point cloud to a PCLPointCloud2
                    pcl::PCLPointCloud2 cloud2;
                    pcl::toPCLPointCloud2(*transformed_cloud, cloud2);

                    // Convert the PCLPointCloud2 to a std::vector<char>
                    std::vector<char> cloud_data(cloud2.data.begin(), cloud2.data.end());

                    auto lidar_frame = new SLAMInMemoryFrame();
                    lidar_frame->FrameSensor = lidar_sensor;
                    lidar_frame->Timestamp.S = timestampS;
                    lidar_frame->Timestamp.Ns = timestampNS;

                    lidar_frame->Data = malloc(cloud_data.size());
                    lidar_frame->SetVariableSize(cloud_data.size());
                    std::copy(cloud_data.data(),
                            cloud_data.data() + cloud_data.size(),
                            reinterpret_cast<char*>(lidar_frame->Data));

                    file.AddFrame(lidar_frame);

                } else {
                    std::cerr << "Failed to extract timestamp from filename: " << filename << std::endl;
                }
            }
        }
    }

    return true;
}


bool loadDARPASubtGroundTruthData(const std::string &dirname, SLAMFile &file) {

    auto gt_sensor = GTSensorBuilder()
                .index(file.Sensors.size())
                .build();

    file.Sensors.AddSensor(gt_sensor);

    std::string line;

    boost::smatch match;
    std::ifstream infile(dirname + "/" + "groundtruth.csv");

    const std::string& num = RegexPattern::number;
    const std::string& start = RegexPattern::start;
    const std::string& end = RegexPattern::end;

    // new format: #ts, x, y, z, qx, qy, qz, qw
    std::string expr = start
                    + num + "\\." + num + ","  // ts in sec.nanosec format
                    + num + ","                // x
                    + num + ","                // y
                    + num + ","                // z
                    + num + ","                // q_x
                    + num + ","                // q_y
                    + num + ","                // q_z
                    + num + "\\s*"             // q_w
                    + end;

    boost::regex groundtruth_line = boost::regex(expr);
    boost::regex comment = boost::regex(RegexPattern::comment);

    // rotate to align camera coordinate
    Eigen::Matrix4f align_mat = (Eigen::Matrix4f() << -1.0,  0.0,  0.0,  0.0,
                                                       0.0,  0.0, -1.0,  0.0,
                                                       0.0, -1.0,  0.0,  0.0,
                                                       0.0,  0.0,  0.0,  1.0).finished();
    Eigen::Matrix4f first_pose_inverse;

    
    bool is_first_pose = true;
    while (std::getline(infile, line)) {
        if (line.empty()) {
            continue;
        } else if (boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, groundtruth_line)) {

            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]);

            float p_RS_R_x = std::stof(match[3]);  
            float p_RS_R_y = std::stof(match[4]);  
            float p_RS_R_z = std::stof(match[5]);  

            float q_RS_x = std::stof(match[6]);  
            float q_RS_y = std::stof(match[7]);  
            float q_RS_z = std::stof(match[8]);  
            float q_RS_w = std::stof(match[9]);

            Eigen::Matrix3f rotationMat = Eigen::Quaternionf(q_RS_w, q_RS_x, q_RS_y, q_RS_z).toRotationMatrix();
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            pose.block(0, 0, 3, 3) = rotationMat;
            pose.block(0, 3, 3, 1) << p_RS_R_x, p_RS_R_y, p_RS_R_z;
            if (is_first_pose) {
                first_pose_inverse = pose.inverse();
                is_first_pose = false;
            }
            pose = align_mat * first_pose_inverse * pose;

            auto gt_frame = new SLAMInMemoryFrame();
            gt_frame->FrameSensor = gt_sensor;
            gt_frame->Timestamp.S = timestampS;
            gt_frame->Timestamp.Ns = timestampNS;
            gt_frame->Data = malloc(gt_sensor->GetFrameSize(gt_frame));

            memcpy(gt_frame->Data, pose.data(), gt_sensor->GetFrameSize(gt_frame));

            file.AddFrame(gt_frame);

            } else {
                std::cerr << "Unknown line:" << line << std::endl;
                return false;
            }
    }
    return true;
}


SLAMFile* DARPASubtReader::GenerateSLAMFile() {
    if(!(lidar || grey || stereo)) {
        std::cerr <<  "No sensors defined\n";
        return nullptr;
    }
    std::string dirname = input;
    std::vector<std::string> requirements = {};

    if (lidar) {
        requirements.emplace_back("/velodyne_pcd");
    }

    if (grey) {
        requirements.emplace_back("cam0");
    }

    if (stereo) {
        requirements.emplace_back("cam1");
    }

    if (gt) {
        requirements.emplace_back("groundtruth.csv");
    }

    if (!checkRequirements(dirname, requirements)) {
        std::cerr << "Invalid folder." << std::endl;
        return nullptr;
    }

    auto slamfilep = new SLAMFile();
    SLAMFile &slamfile = *slamfilep;

    int rate = 10;
    int width = 720;
    int height = 536;

    CameraSensor::intrinsics_t cam0_intrinsics;
    CameraSensor::intrinsics_t cam1_intrinsics;

    CameraSensor::distortion_type_t cam_distortion_type;
    CameraSensor::distortion_coefficients_t cam0_distortion;
    CameraSensor::distortion_coefficients_t cam1_distortion;

    Sensor::pose_t cam0_pose = Eigen::Matrix4f::Identity();
    Sensor::pose_t cam1_pose = Eigen::Matrix4f::Identity();

    bool rect = true;
    
    // Check the raw data type
    if (dirname.find("anymal1") != std::string::npos && dirname.find("sync") != std::string::npos) {

        std::cout << "Using rectified parameter of ANYMAL1 cam0 and cam1" << std::endl;
        get_DARPSSubt_params(cam0_intrinsics, cam1_intrinsics, cam_distortion_type, cam0_distortion, cam1_distortion);

        cam1_pose(0, 3) = 0.09939706071873089;
        
        rect = true;

    }

    if (grey && !loadDARPASubtGreyData(dirname + "/cam0", slamfile, rate, width, height, cam0_pose, cam0_intrinsics, cam_distortion_type, cam0_distortion)) {
        std::cout << "Error while loading Left Grey information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    if (grey && stereo && !loadDARPASubtGreyData(dirname + "/cam1", slamfile, rate, width, height, cam1_pose, cam1_intrinsics, cam_distortion_type, cam1_distortion)) {
        std::cout << "Error while loading Right Grey information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    if (lidar && !loadDARPASubtLidarData(dirname, slamfile)) {
        std::cout << "Error while loading LiDAR information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    if (!imu) std::cout << "IMU sensor disabled by default, check DARPASubt.h" << std::endl;
    if (stereo && imu && !loadDARPASubtIMUData(dirname, slamfile)) {
        std::cout << "Error while loading Ouster IMU information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    if (gt && !loadDARPASubtGroundTruthData(dirname, slamfile)) {
        std::cout << "Error while loading Ground Truth information." << std::endl;
        delete slamfilep;
        return nullptr;
    }
    
    return slamfilep;

}

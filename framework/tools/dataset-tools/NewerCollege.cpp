/*

 Copyright (c) 2023 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

*/

#include "include/NewerCollege.h"
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

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <cmath>
#include "TimeStamp.h"


using namespace slambench::io;

bool loadNewerLidarData(const std::string &dirname, SLAMFile &file) {
    
    std::string lidar_dirname = dirname + "/ouster_scan";
    auto lidar_sensor = new LidarSensor(lidar_dirname);
    lidar_sensor->Index = file.Sensors.size();
    lidar_sensor->Rate = 10.0;
    lidar_sensor->BeamNum = 64;

    file.Sensors.AddSensor(lidar_sensor);
    std::cout << "Lidar sensor created... (It would take a while)" << std::endl;

    Eigen::Matrix4f Z_rot_135;
    float val = std::sqrt(2.0f) / 2.0f;

    Z_rot_135 << -val, -val, 0, 0,
                  val, -val, 0, 0,
                  0,    0,   1, 0,
                  0,    0,   0, 1;

    // Try opening the timestamp file
    std::ifstream infile(lidar_dirname + "/timestamp.txt");
    if (infile.is_open()) { // If timestamp file exists
        // TODO: if there is a timestamp file
        infile.close();

    } else { // If timestamp file does not exist, infer from .pcd filenames

        boost::filesystem::directory_iterator end_iter;
        boost::regex pattern("cloud_(\\d+)_(\\d+).pcd");
        boost::smatch match;

        for (boost::filesystem::directory_iterator dir_iter(lidar_dirname); dir_iter != end_iter; ++dir_iter) {
            
            if (boost::filesystem::is_regular_file(dir_iter->status()) && dir_iter->path().extension() == ".pcd") {

                std::string filename = dir_iter->path().filename().string();

                if (boost::regex_match(filename, match, pattern)) {

                    // Extract timestamp from the filename
                    int timestampS = std::stoi(match[1].str());
                    int timestampNS = std::stoi(match[2].str());

                    // ====== Load pointcloud for each frame
                    std::string lidar_file_pcd = lidar_dirname + "/" + filename;

                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

                    // Read the PCD file into the point cloud
                    if (pcl::io::loadPCDFile<pcl::PointXYZI>(lidar_file_pcd.c_str(), *cloud) == -1) {
                        std::cerr << "Failed to read file: " << lidar_file_pcd.c_str() << std::endl;
                        return false;
                    }

                    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
                    pcl::transformPointCloud(*cloud, *transformed_cloud, Z_rot_135);

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


bool loadNewerGroundTruthData(const std::string &dirname, SLAMFile &file) {

    auto gt_sensor = GTSensorBuilder()
                .index(file.Sensors.size())
                .build();

    file.Sensors.AddSensor(gt_sensor);

    std::string line;

    boost::smatch match;
    std::ifstream infile(dirname + "/" + "poses.csv");

    const std::string& num = RegexPattern::number;
    const std::string& start = RegexPattern::start;
    const std::string& end = RegexPattern::end;

    // new format: #sec, nansec, x, y, z, qx, qy, qz, qw
    std::string expr = start
        + num + ","             // sec timestamp
        + num + ","             // nanosecond timestamp
        + num + ","             // x
        + num + ","             // y
        + num + ","             // z
        + num + ","             // qx
        + num + ","             // qy
        + num + ","             // qz
        + num + "\\s*"          // qw
        + end;

    boost::regex groundtruth_line = boost::regex(expr);
    boost::regex comment = boost::regex(RegexPattern::comment);

    Eigen::Matrix4f align_mat = (Eigen::Matrix4f() <<   0.0, -1.0,  0.0, 0.0,
                                                        0.0,  0.0, -1.0, 0.0,
                                                        1.0,  0.0,  0.0, 0.0,
                                                        0.0,  0.0,  0.0, 1.0).finished();
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


SLAMFile* NewerCollegeReader::GenerateSLAMFile() {
    if(!(lidar)) {
        std::cerr <<  "No sensors defined\n";
        return nullptr;
    }
    std::string dirname = input;
    std::vector<std::string> requirements = {};

    if (lidar) {
        requirements.emplace_back("/ouster_scan");
    }

    if (!checkRequirements(dirname, requirements)) {
        std::cerr << "Invalid folder." << std::endl;
        return nullptr;
    }

    auto slamfilep = new SLAMFile();
    SLAMFile &slamfile = *slamfilep;

    if (lidar && !loadNewerLidarData(dirname, slamfile)) {
        std::cout << "Error while loading Lidar information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    if (gt && !loadNewerGroundTruthData(dirname, slamfile)) {
        std::cout << "Error while loading Ground Truth information." << std::endl;
        delete slamfilep;
        return nullptr;
    }
    
    return slamfilep;

}

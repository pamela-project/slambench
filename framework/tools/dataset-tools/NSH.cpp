/*

 Copyright (c) 2023 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

*/

#include "include/NSH.h"
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

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include "TimeStamp.h"


using namespace slambench::io;

std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(' ');
    if (first == std::string::npos)
        return "";  // Empty string or only whitespace
    size_t last = str.find_last_not_of(' ');
    return str.substr(first, last - first + 1);
}


bool loadNSHLidarData(const std::string &dirname,
                        SLAMFile &file) {
    
    auto lidar_sensor = new LidarSensor(dirname);
    lidar_sensor->Index = file.Sensors.size();
    lidar_sensor->Rate = 10.0;
    lidar_sensor->BeamNum = 16;

    file.Sensors.AddSensor(lidar_sensor);
    std::cout << "Lidar sensor created ..." << std::endl;

    std::string line;
    std::ifstream infile(dirname + "/timestamp.txt");

    boost::smatch match;
    boost::regex comment = boost::regex(RegexPattern::comment);
    boost::regex pattern("^(\\d+)\\.(\\d+)$");

    int lidar_index = 0;
    while (std::getline(infile, line)) {
        line = trim(line);

        if (line.empty()) {
            continue;
        }

        size_t pos = line.find('.');
        if (pos != std::string::npos) {
            std::string before_dot = line.substr(0, pos);
            std::string after_dot = line.substr(pos + 1);
            int timestampS, timestampNS;
            try {
                timestampS = std::stoi(before_dot);
                timestampNS = std::stoi(after_dot);
            } catch (const std::invalid_argument& e) {
                std::cerr << "Invalid argument: " << e.what() << std::endl;
            } catch (const std::out_of_range& e) {
                std::cerr << "Out of range: " << e.what() << std::endl;
            }

            // ====== Load pointcloud for each frame
            // start from 0000000000.bin
            std::stringstream tmp_filename;
            tmp_filename << std::setw(10) << std::setfill('0') << lidar_index;
            std::string lidar_file_pcd = tmp_filename.str() + ".pcd";
            lidar_file_pcd = dirname + "/" + lidar_file_pcd;
            lidar_index++;

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

            // Read the PCD file into the point cloud
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(lidar_file_pcd.c_str(), *cloud) == -1)
            {
                std::cerr << "Failed to read file: " << lidar_file_pcd.c_str() << std::endl;
                return -1;
            }

            // Convert the point cloud to a PCLPointCloud2
            pcl::PCLPointCloud2 cloud2;
            pcl::toPCLPointCloud2(*cloud, cloud2);

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
            std::cerr << "Unknown line:" << line << std::endl;
            infile.close();
            return false;
        }
    }
    infile.close();
    return true;
}

SLAMFile* NSHReader::GenerateSLAMFile() {
    if(!(lidar)) {
        std::cerr <<  "No sensors defined\n";
        return nullptr;
    }
    std::string dirname = input;
    std::vector<std::string> requirements = {};

    if (lidar) {
        requirements.emplace_back("0000000000.pcd");
        requirements.emplace_back("timestamp.txt");
    }

    if (!checkRequirements(dirname, requirements)) {
        std::cerr << "Invalid folder." << std::endl;
        return nullptr;
    }

    auto slamfilep = new SLAMFile();
    SLAMFile &slamfile = *slamfilep;

    if (lidar && !loadNSHLidarData(dirname, slamfile)) {
        std::cout << "Error while loading Lidar information." << std::endl;
        delete slamfilep;
        return nullptr;
    }
    
    return slamfilep;

}
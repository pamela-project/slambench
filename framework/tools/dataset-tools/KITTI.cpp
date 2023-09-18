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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>

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

int kitti_start = 0;
int kitti_end = 0;

std::list<slambench::TimeStamp> loadLeftGreyTimeStamps(const std::string &dirname) {

    std::string line;
    std::ifstream infile(dirname + "/image_00/timestamps.txt");
    std::list<slambench::TimeStamp> timestamps;

    boost::smatch match;
    boost::regex comment = boost::regex(RegexPattern::comment);
    // format of timestamp: yyyy-mm-dd hr:min:sec.nsec 2011-09-30 12:40:59.442522880
    // extract hr, min, sec, nsec
    boost::regex pattern("^\\d{4}-\\d{2}-\\d{2} (\\d{2}):(\\d{2}):(\\d{2})\\.(\\d{9})$");

    int ts_index = 0;
    while (std::getline(infile, line)) {
        if (line.empty() || boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, pattern)) {

            if (ts_index < kitti_start) {
                ts_index++;
                continue;
            }
            ts_index++;

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
            infile.close();
            timestamps.clear();
            return timestamps;
        }
    }
    infile.close();
    return timestamps;
}

bool resizeKITTIImage(const std::string &filename, int width, int height) {

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
}

bool loadKITTIRGBData(const std::string &dirname,
                      const std::string &camera_idx,
                      SLAMFile &file,
                      const Sensor::pose_t &pose,
                      const CameraSensor::intrinsics_t &intrinsics,
                      const CameraSensor::distortion_type_t &distortion_type,
                      const CameraSensor::distortion_coefficients_t &distortion,
                      const bool rect) {
    
    auto img_params = KITTIReader::get_image_params(rect);
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
    std::ifstream infile(dirname + "/" + camera_idx + "/timestamps.txt");

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

            if (img_index < kitti_start) {
                img_index++;
                continue;
            }

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
            frame_name << dirname << "/" << camera_idx << "/data/" << rgb_filename;
            // rectified image should be resize to dimensions that is multiple of 16
            if (rect) {
                if (!resizeKITTIImage(frame_name.str(), img_params.width, img_params.height)){
                    return false;
                }
            }
            rgb_frame->filename = frame_name.str();

            if (access(rgb_frame->filename.c_str(), F_OK) < 0) {
                printf("No RGB image for frame (%s)\n", frame_name.str().c_str());
                perror("");
                return false;
            }

            file.AddFrame(rgb_frame);

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            infile.close();
            return false;
        }
    }
    infile.close();
    return true;
}

bool loadKITTIGreyData(const std::string &dirname,
                      const std::string &camera_idx,
                      SLAMFile &file,
                      const Sensor::pose_t &pose,
                      const CameraSensor::intrinsics_t &intrinsics,
                      const CameraSensor::distortion_type_t &distortion_type,
                      const CameraSensor::distortion_coefficients_t &distortion,
                      const bool rect) {
    
    auto img_params = KITTIReader::get_image_params(rect);
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
    std::ifstream infile(dirname + "/" + camera_idx + "/timestamps.txt");
    // std::ifstream infile(dirname + "/image_00/timestamps.txt");

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

            if (img_index < kitti_start) {
                img_index++;
                continue;
            }

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
            frame_name << dirname << "/" << camera_idx << "/data/" << grey_filename;
            if (rect) {
                if (!resizeKITTIImage(frame_name.str(), img_params.width, img_params.height)){
                    infile.close();
                    return false;
                }
            }
            grey_frame->filename = frame_name.str();

            if (access(grey_frame->filename.c_str(), F_OK) < 0) {
                printf("No Grey image for frame (%s)\n", frame_name.str().c_str());
                perror("");
                infile.close();
                return false;
            }

            file.AddFrame(grey_frame);

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            infile.close();
            return false;
        }
    }
    infile.close();
    return true;
}

bool loadKITTIIMUData(const std::string &dirname,
                      SLAMFile &file,
                      const Sensor::pose_t &pose) {

    auto imu_sensor = new IMUSensor(dirname);
    imu_sensor->Index = file.Sensors.size();
    imu_sensor->Rate = 10.0;

    imu_sensor->GyroscopeNoiseDensity = 1.6968e-04;
    imu_sensor->GyroscopeBiasDiffusion = 0.003491;
    imu_sensor->AcceleratorNoiseDensity = 2.0000e-3;
    imu_sensor->AcceleratorBiasDiffusion = 5.0000e-3;
    imu_sensor->CopyPose(pose);

    file.Sensors.AddSensor(imu_sensor);

    const std::string& num = RegexPattern::number;
    const std::string& start = RegexPattern::start;
    const std::string& end = RegexPattern::end;

    std::string expr = start
        + num + "\\s+" + num + "\\s+" + num + "\\s+"           // lat, lon, alt: 1-3
        + num + "\\s+" + num + "\\s+" + num + "\\s+"           // roll, pitch, yaw: 4-6
        + num + "\\s+" + num + "\\s+"                         // vn, ve: 7-8
        + num + "\\s+" + num + "\\s+" + num + "\\s+"           // vf, vl, vu: 9-11
        + num + "\\s+" + num + "\\s+" + num + "\\s+"           // ax, ay, az: 12-14
        + num + "\\s+" + num + "\\s+" + num + "\\s+"           // af, al, au: 15-17
        + num + "\\s+" + num + "\\s+" + num + "\\s+"           // wx, wy, wz: 18-20
        + num + "\\s+" + num + "\\s+" + num + "\\s+"           // wf, wl, wu: 21-23
        + num + "\\s+" + num + "\\s+"                         // pos_accuracy, vel_accuracy: 24-25
        + num + "\\s+" + num + "\\s+"                         // navstat, numsats: 26-27
        + num + "\\s+" + num + "\\s+" + num + "\\s*"          // posmode, velmode, orimode: 28-30
        + end;

    std::string line_ts;
    std::string line_imu;

    boost::smatch match_ts;
    boost::smatch match_imu;
    std::ifstream infile_ts(dirname + "/oxts/timestamps.txt");
    std::ifstream infile_imu;

    boost::regex comment = boost::regex(RegexPattern::comment);
    boost::regex pattern_imu = boost::regex(expr);
    boost::regex pattern_ts("^\\d{4}-\\d{2}-\\d{2} (\\d{2}):(\\d{2}):(\\d{2})\\.(\\d{9})$");

    int imu_index = 0;
    bool lock = true;
    while (std::getline(infile_ts, line_ts)) {
        if (line_ts.empty()) {
            continue;
        } else if (boost::regex_match(line_ts, match_ts, comment)) {
            continue;
        } else if (boost::regex_match(line_ts, match_ts, pattern_ts)) {

            if (imu_index < kitti_start) {
                imu_index++;
                continue;
            }

            int hour = std::stoi(match_ts[1]);
            int min = std::stoi(match_ts[2]);
            int second = std::stoi(match_ts[3]);
            int timestampS = hour * 3600 + min * 60 + second;
            int timestampNS = std::stoi(match_ts[4]) * std::pow(10, 9 - match_ts[4].length());

            auto IMU_frame = new SLAMInMemoryFrame();
            IMU_frame->FrameSensor = imu_sensor;
            IMU_frame->Timestamp.S = timestampS;
            IMU_frame->Timestamp.Ns = timestampNS;
            IMU_frame->Data = malloc(imu_sensor->GetFrameSize(IMU_frame));

            // start from 0000000000.txt
            std::stringstream tmp_filename;
            tmp_filename << std::setw(10) << std::setfill('0') << imu_index;
            std::string imu_filename = tmp_filename.str() + ".txt";
            imu_index++;
            infile_imu.open(dirname + "/oxts/data/" + imu_filename);
            std::getline(infile_imu, line_imu);

            if (boost::regex_match(line_imu, match_imu, pattern_imu)) {
                float gx = std::stof(match_imu[18]);
                float gy = std::stof(match_imu[19]);
                float gz = std::stof(match_imu[20]);
                float ax = std::stof(match_imu[12]);
                float ay = std::stof(match_imu[13]);
                float az = std::stof(match_imu[14]);

                ((float *)IMU_frame->Data)[0] = gx;
                ((float *)IMU_frame->Data)[1] = gy;
                ((float *)IMU_frame->Data)[2] = gz;
                ((float *)IMU_frame->Data)[3] = ax;
                ((float *)IMU_frame->Data)[4] = ay;
                ((float *)IMU_frame->Data)[5] = az;

                file.AddFrame(IMU_frame);
                infile_imu.close();
            } else {
                std::cerr << "Unknown line:" << line_imu << std::endl;
                infile_imu.close();
                infile_ts.close();
                return false;
            }

        } else {
            std::cerr << "Unknown line:" << line_ts << std::endl;
            infile_ts.close();
            return false;
        }
    }
    infile_ts.close();
    return true;
}

bool loadKITTILidarData(const std::string &dirname,
                        SLAMFile &file,
                        const Sensor::pose_t &pose) {
    
    auto lidar_sensor = new LidarSensor(dirname);
    lidar_sensor->Index = file.Sensors.size();
    lidar_sensor->Rate = 10.0;

    lidar_sensor->PointsPerCycle = 100000;
    lidar_sensor->AltitudeAboveGround = 1.73;
    lidar_sensor->HorizontalFoV[0] = 0.0;
    lidar_sensor->HorizontalFoV[1] = 360.0;
    lidar_sensor->VerticalFoV[0] = 2.0;
    lidar_sensor->VerticalFoV[1] = -24.9;
    lidar_sensor->VerticalAngResolution = 0.09;
    lidar_sensor->BeamNum = 64;
    lidar_sensor->CopyPose(pose);

    file.Sensors.AddSensor(lidar_sensor);
    std::cout << "LiDAR sensor created..." << std::endl;

    std::string line;
    std::ifstream infile(dirname + "/velodyne_points/timestamps.txt");

    boost::smatch match;
    boost::regex comment = boost::regex(RegexPattern::comment);
    // format of timestamp: yyyy-mm-dd hr:min:sec.nsec 2011-09-30 12:40:59.442522880
    // extract hr, min, sec, nsec
    boost::regex pattern("^\\d{4}-\\d{2}-\\d{2} (\\d{2}):(\\d{2}):(\\d{2})\\.(\\d{9})$");

    int lidar_index = 0;
    int num_point = lidar_sensor->PointsPerCycle;
    while (std::getline(infile, line)) {
        if (line.empty()) {
            continue;
        } else if (boost::regex_match(line, match, comment)) {
            continue;
        } else if (boost::regex_match(line, match, pattern)) {

            if (lidar_index < kitti_start) {
                lidar_index++;
                continue;
            }

            int hour = std::stoi(match[1]);
            int min = std::stoi(match[2]);
            int second = std::stoi(match[3]);
            int timestampS = hour * 3600 + min * 60 + second;
            int timestampNS = std::stoi(match[4]) * std::pow(10, 9 - match[4].length());

            // ====== Load pointcloud for each frame
            // start from 0000000000.bin
            std::stringstream tmp_filename;
            tmp_filename << std::setw(10) << std::setfill('0') << lidar_index;
            std::string lidar_file_bin = tmp_filename.str() + ".bin";
            lidar_file_bin = dirname + "/velodyne_points/data/" + lidar_file_bin;
            lidar_index++;

            std::ifstream stream(lidar_file_bin.c_str(), std::ios::binary);
            if (!stream) {
                std::cerr << "Failed to open the file.\n";
                return false;
            }

            // Get the size of the file
            stream.seekg(0, std::ios::end);
            size_t fileSize = stream.tellg();
            stream.seekg(0, std::ios::beg);

            std::vector<char> data_vector(fileSize);
            stream.read(data_vector.data(), fileSize);
            size_t actual_num_point = stream.gcount() / (sizeof(float) * 4);
            stream.close();

            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

            for (size_t i = 0; i < actual_num_point; ++i) {
                pcl::PointXYZI point;

                // Assuming your data is in little endian format
                point.x = *reinterpret_cast<float*>(&data_vector[i * sizeof(float) * 4]);
                point.y = *reinterpret_cast<float*>(&data_vector[i * sizeof(float) * 4 + sizeof(float)]);
                point.z = *reinterpret_cast<float*>(&data_vector[i * sizeof(float) * 4 + 2 * sizeof(float)]);
                point.intensity = *reinterpret_cast<float*>(&data_vector[i * sizeof(float) * 4 + 3 * sizeof(float)]);

                cloud->push_back(point);
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
     * 03: 2011_09_26_drive_0067 000000 000800  NOT PUBLIC
     * 04: 2011_09_30_drive_0016 000000 000270
     * 05: 2011_09_30_drive_0018 000000 002760
     * 06: 2011_09_30_drive_0020 000000 001100
     * 07: 2011_09_30_drive_0027 000000 001100
     * 08: 2011_09_30_drive_0028 001100 005170
     * 09: 2011_09_30_drive_0033 000000 001590
     * 10: 2011_09_30_drive_0034 000000 001200
    */
    std::ifstream infile;
    
    if (dirname.find("2011_10_03_drive_0027") != std::string::npos) {
        // pose 04
        std::cerr << "Use pose 00 for 2011_10_03_drive_0027" << std::endl;
        infile.open(dirname+"/poses/00.txt");
        kitti_start = 0; kitti_end = 4540;

    } else if (dirname.find("2011_10_03_drive_0042") != std::string::npos) {
        // pose 04
        std::cerr << "Use pose 00 for 2011_10_03_drive_0042" << std::endl;
        infile.open(dirname+"/poses/01.txt");
        kitti_start = 0; kitti_end = 1100;

    } else if (dirname.find("2011_10_03_drive_0034") != std::string::npos) {
        // pose 04
        std::cerr << "Use pose 00 for 2011_10_03_drive_0034" << std::endl;
        infile.open(dirname+"/poses/02.txt");
        kitti_start = 0; kitti_end = 4660;

    } else if (dirname.find("2011_09_30_drive_0016") != std::string::npos) {
        // pose 04
        std::cerr << "Use pose 04 for 2011_09_30_drive_0016" << std::endl;
        infile.open(dirname+"/poses/04.txt");
        kitti_start = 0; kitti_end = 270;

    } else if (dirname.find("2011_09_30_drive_0018") != std::string::npos) {
        // pose 05
        std::cerr << "Use pose 05 for 2011_09_30_drive_0018" << std::endl;
        infile.open(dirname+"/poses/05.txt");
        kitti_start = 0; kitti_end = 2760;

    } else if (dirname.find("2011_09_30_drive_0020") != std::string::npos) {
        // pose 06
        std::cerr << "Use pose 06 for 2011_09_30_drive_0020" << std::endl;
        infile.open(dirname+"/poses/06.txt");
        kitti_start = 0; kitti_end = 1100;

    } else if (dirname.find("2011_09_30_drive_0027") != std::string::npos) {
        // pose 07
        std::cerr << "Use pose 07 for 2011_09_30_drive_0027" << std::endl;
        infile.open(dirname+"/poses/07.txt");
        kitti_start = 0; kitti_end = 1100;

    } else if (dirname.find("2011_09_30_drive_0028") != std::string::npos) {
        // pose 08
        std::cerr << "Use pose 08 for 2011_09_30_drive_0028" << std::endl;
        infile.open(dirname+"/poses/08.txt");
        kitti_start = 1100; kitti_end = 5170;

    } else if (dirname.find("2011_09_30_drive_0033") != std::string::npos) {
        // pose 09
        std::cerr << "Use pose 09 for 2011_09_30_drive_0033" << std::endl;
        infile.open(dirname+"/poses/09.txt");
        kitti_start = 0; kitti_end = 1590;

    } else if (dirname.find("2011_09_30_drive_0034") != std::string::npos) {
        // pose 10
        std::cerr << "Use pose 10 for 2011_09_30_drive_0034" << std::endl;
        infile.open(dirname+"/poses/10.txt");
        kitti_start = 0; kitti_end = 1200;

    } else {
        std::cerr << "Invalid path to KITTI dataset groundtruth" << std::endl;
        return false;
    }

    if (!infile.is_open()) {
        std::cerr << "Fail to open the pose file" << std::endl;
    }

    std::list<slambench::TimeStamp> timestamps = loadLeftGreyTimeStamps(dirname);
    if (timestamps.size() == 0) {
        std::cerr << "Unable to load timestamps of left RGB camera" << std::endl;
        infile.close();
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
                infile.close();
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
    infile.close();
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

    if (gt) {
        requirements.emplace_back("poses");
    }

    if (!checkRequirements(dirname, requirements)) {
        std::cerr << "Invalid folder." << std::endl;
        return nullptr;
    }

    auto slamfilep = new SLAMFile();
    SLAMFile &slamfile = *slamfilep;

    CameraSensor::intrinsics_t cam_intrinsics_lgrey;
    CameraSensor::intrinsics_t cam_intrinsics_rgrey;
    CameraSensor::intrinsics_t cam_intrinsics_lrgb;
    CameraSensor::intrinsics_t cam_intrinsics_rrgb;

    CameraSensor::distortion_type_t cam_distortion_type;
    CameraSensor::distortion_coefficients_t cam_distortion_lgrey;
    CameraSensor::distortion_coefficients_t cam_distortion_rgrey;
    CameraSensor::distortion_coefficients_t cam_distortion_lrgb;
    CameraSensor::distortion_coefficients_t cam_distortion_rrgb;

    Sensor::pose_t pose_lgrey = Eigen::Matrix4f::Identity();
    Sensor::pose_t pose_rgrey = Eigen::Matrix4f::Identity();
    Sensor::pose_t pose_lrgb = Eigen::Matrix4f::Identity();
    Sensor::pose_t pose_rrgb = Eigen::Matrix4f::Identity();

    // a_2_b the transformation matrix that convert point in a coordinate to b coordinate
    Sensor::pose_t imu_2_velo = Eigen::Matrix4f::Identity();
    Sensor::pose_t velo_2_lgrey = Eigen::Matrix4f::Identity();

    Sensor::pose_t R_rect_00;

    bool rect = true;

    KITTIReader::DatasetOrigin d_origin = check_data_origin();
    
    // Check the raw data type
    if (d_origin == KITTIReader::DatasetOrigin::RD11_09_30_RECT) {

        std::cout << "Using rectified parameter from 2011-09-30" << std::endl;
        get_params(cam_intrinsics_lgrey, cam_intrinsics_rgrey, cam_intrinsics_lrgb, cam_intrinsics_rrgb,
                    cam_distortion_type, cam_distortion_lgrey, cam_distortion_rgrey, cam_distortion_lrgb, cam_distortion_rrgb);

        pose_rgrey(0, 3) = -5.370000e-01;  
        pose_rgrey = pose_rgrey.inverse().eval();

        pose_lrgb(0, 3) = 5.956621e-02;
        pose_lrgb = pose_lrgb.inverse().eval();

        pose_rrgb(0, 3) = -4.731050e-01;
        pose_rrgb = pose_rrgb.inverse().eval();

        imu_2_velo <<  9.999976e-01,  7.553071e-04, -2.035826e-03, -8.086759e-01, 
                      -7.854027e-04,  9.998898e-01, -1.482298e-02,  3.195559e-01,
                       2.024406e-03,  1.482454e-02,  9.998881e-01, -7.997231e-01,
                       0.000000e+00,  0.000000e+00,  0.000000e+00,  1.000000e+00;

        velo_2_lgrey << 7.027555e-03, -9.999753e-01,  2.599616e-05, -7.137748e-03,  
                       -2.254837e-03, -4.184312e-05, -9.999975e-01, -7.482656e-02,
                        9.999728e-01,  7.027479e-03, -2.255075e-03, -3.336324e-01,
                        0.000000e+00,  0.000000e+00,  0.000000e+00,  1.000000e+00;

        R_rect_00 << 9.999280e-01, 8.085985e-03, -8.866797e-03, 0.000000e+00,
                    -8.123205e-03, 9.999583e-01, -4.169750e-03, 0.000000e+00,
                     8.832711e-03, 4.241477e-03,  9.999520e-01, 0.000000e+00,
                     0.000000e+00, 0.000000e+00,  0.000000e+00, 1.000000e+00;
        
        rect = true;

    } else if (d_origin == KITTIReader::DatasetOrigin::RD11_10_03_RECT) {

        std::cout << "Using rectified parameter from 2011-10-03" << std::endl;
        get_params(cam_intrinsics_lgrey, cam_intrinsics_rgrey, cam_intrinsics_lrgb, cam_intrinsics_rrgb,
                    cam_distortion_type, cam_distortion_lgrey, cam_distortion_rgrey, cam_distortion_lrgb, cam_distortion_rrgb);

        pose_rgrey(0, 3) = -5.370000e-01;  
        pose_rgrey = pose_rgrey.inverse().eval();

        pose_lrgb(0, 3) = 5.954406e-02;
        pose_lrgb = pose_lrgb.inverse().eval();

        pose_rrgb(0, 3) = -4.738786e-01;
        pose_rrgb = pose_rrgb.inverse().eval();

        imu_2_velo <<  9.999976e-01,  7.553071e-04, -2.035826e-03, -8.086759e-01, 
                      -7.854027e-04,  9.998898e-01, -1.482298e-02,  3.195559e-01,
                       2.024406e-03,  1.482454e-02,  9.998881e-01, -7.997231e-01,
                       0.000000e+00,  0.000000e+00,  0.000000e+00,  1.000000e+00;

        velo_2_lgrey <<  7.967514e-03, -9.999679e-01, -8.462264e-04, -1.377769e-02, 
                        -2.771053e-03,  8.241710e-04, -9.999958e-01, -5.542117e-02,
                         9.999644e-01,  7.969825e-03, -2.764397e-03, -2.918589e-01,
                         0.000000e+00,  0.000000e+00,  0.000000e+00,  1.000000e+00;

        R_rect_00 << 9.999454e-01, 7.259129e-03, -7.519551e-03, 0.000000e+00,
                    -7.292213e-03, 9.999638e-01, -4.381729e-03, 0.000000e+00,
                     7.487471e-03, 4.436324e-03,  9.999621e-01, 0.000000e+00,
                     0.000000e+00, 0.000000e+00,  0.000000e+00, 1.000000e+00;
        
        rect = true;

    } else {

        std::cout << "Invalid Path, please check d_origin at KITTI.cpp" << std::endl;
        return nullptr;

    }

    if (gt && !loadKITTIGroundTruthData(dirname, slamfile)) {
        std::cout << "Error while loading GroundTruth information." << std::endl;
        delete slamfilep;
        return nullptr;
    }
    
    // Load Left Grey Camera
    std::string left_grey = "image_00";
    if (grey && !loadKITTIGreyData(dirname, left_grey, slamfile, pose_lgrey, 
                                   cam_intrinsics_lgrey, cam_distortion_type, cam_distortion_lgrey, rect)) {
        std::cout << "Error while loading left Grey information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    // Load Right Grey Camera
    std::string right_grey = "image_01";
    if (grey && stereo && !loadKITTIGreyData(dirname, right_grey, slamfile, pose_rgrey, 
                                   cam_intrinsics_rgrey, cam_distortion_type, cam_distortion_rgrey, rect)) {
        std::cout << "Error while loading right Grey information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    // Load Left RGB Camera
    std::string left_rgb = "image_02";
    if (rgb && !loadKITTIRGBData(dirname, left_rgb, slamfile, pose_lrgb, 
                                 cam_intrinsics_lrgb, cam_distortion_type, cam_distortion_lrgb, rect)) {
        std::cout << "Error while loading left RGB information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    // Load Right RGB Camera
    std::string right_rgb = "image_03";
    if (rgb && stereo && !loadKITTIRGBData(dirname, right_rgb, slamfile, pose_rrgb, 
                                 cam_intrinsics_rrgb, cam_distortion_type, cam_distortion_rrgb, rect)) {
        std::cout << "Error while loading left RGB information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    // do not support unrectified and unsync imu and lidar data
    if (!imu) std::cout << "IMU sensor disabled by default, check KITTI.h" << std::endl;
    Sensor::pose_t pose_imu = imu_2_velo;
    if (imu && !loadKITTIIMUData(dirname, slamfile, pose_imu)) {
        std::cout << "Error while loading IMU information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    Sensor::pose_t pose_lidar = velo_2_lgrey;
    if (lidar && rect) {
        std::cout << "LiDAR pose represents transformation of a point from LiDAR coordinates to rectified camera coordinate!" << std::endl;
        pose_lidar = R_rect_00 * velo_2_lgrey;
    }
    if (lidar && !loadKITTILidarData(dirname, slamfile, pose_lidar)) {
        std::cout << "Error while loading LiDAR information." << std::endl;
        delete slamfilep;
        return nullptr;
    }
    
    return slamfilep;

}
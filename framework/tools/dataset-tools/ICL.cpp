/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "include/ICL.h"

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

using namespace slambench::io ;

static DepthSensor *GetDepthSensor(const Sensor::pose_t &pose, const DepthSensor::intrinsics_t &intrinsics, const DepthSensor::disparity_params_t &dparams, const DepthSensor::disparity_type_t &dtype) {
    DepthSensor *sensor = new DepthSensor("Depth");
    sensor->Index = 0;
    sensor->Width = 640;
    sensor->Height = 480;
    sensor->FrameFormat = frameformat::Raster;
    sensor->PixelFormat = pixelformat::D_I_16;
    sensor->DisparityType = dtype;
    sensor->Description = "Depth";
    sensor->Rate = 1;
    sensor->CopyPose(pose);
    sensor->CopyIntrinsics(intrinsics);
    sensor->CopyDisparityParams(dparams);

    return sensor;
}


static CameraSensor *GetRGBSensor(const Sensor::pose_t &pose, const CameraSensor::intrinsics_t &intrinsics) {
    CameraSensor *sensor = new CameraSensor("RGB", CameraSensor::kCameraType);
    sensor->Index = 0;
    sensor->Width = 640;
    sensor->Height = 480;
    sensor->FrameFormat = frameformat::Raster;
    sensor->PixelFormat = pixelformat::RGB_III_888;
    sensor->Description = "RGB";
    sensor->Rate = 1;

    sensor->CopyPose(pose);
    sensor->CopyIntrinsics(intrinsics);

    return sensor;
}

static CameraSensor *GetGreySensor(const Sensor::pose_t &pose, const CameraSensor::intrinsics_t &intrinsics) {
    CameraSensor *sensor = new CameraSensor("Grey", CameraSensor::kCameraType);
    sensor->Index = 0;
    sensor->Width = 640;
    sensor->Height = 480;
    sensor->FrameFormat = frameformat::Raster;
    sensor->PixelFormat = pixelformat::G_I_8;
    sensor->Description = "Grey";
    sensor->Rate = 1;

    sensor->CopyPose(pose);
    sensor->CopyIntrinsics(intrinsics);

    return sensor;
}


void undistort_frame(slambench::io::SLAMFileFrame *frame, void *data) {
    uint16_t *depthMap = (uint16_t*)data;

    uint32_t w = ((slambench::io::CameraSensor*)frame->FrameSensor)->Width;
    uint32_t h = ((slambench::io::CameraSensor*)frame->FrameSensor)->Height;

    float u0 = 320.00;
    float v0 = 240.00;
    float fx = -600.00;
    float fy = 600.00;

    for (uint32_t v = 0; v < h; v++) {
        for (uint32_t u = 0; u < w; u++) {
            double u_u0_by_fx = (u - u0) / fx;
            double v_v0_by_fy = (v - v0) / fy;

            depthMap[u + v * w] = depthMap[u + v * w] / std::sqrt(u_u0_by_fx * u_u0_by_fx
                                                                  + v_v0_by_fy * v_v0_by_fy
                                                                  + 1);
        }
    }
}

/*
 *
 * The dataset folder contains :
 * > accelerometer.txt  depth  depth.txt  groundtruth.txt  rgb  rgb.txt
 *
 */

bool analyseIclFolder(const std::string &dirname) {

    static const std::vector<std::string> requirements = {
            //"accelerometer.txt",
            //"rgb.txt",
            //"rgb",
            //"depth.txt",
            //"depth",
            //"groundtruth.txt"
            //"cam0.ccam",
            //"cam0_gt.visim"
    };

    try {
        if ( !boost::filesystem::exists( dirname ) ) return false;

        boost::filesystem::directory_iterator end_itr; // default construction yields past-the-end
        for ( auto requirement : requirements ) {
            bool seen = false;

            for ( boost::filesystem::directory_iterator itr( dirname ); itr != end_itr; ++itr ) {
                if (requirement == itr->path().filename()) {
                    seen = true;
                }
            }

            if (!seen) {
                std::cout << "File not found: <dataset_dir>/" << requirement << std::endl;
                return false;
            }
        }
    } catch (boost::filesystem::filesystem_error& e)  {
        std::cerr << "I/O Error with directory " << dirname << std::endl;
        std::cerr << e.what() << std::endl;
        return false;
    }

    return true;
}


bool loadICLDepthData(const std::string &dirname , SLAMFile &file, const Sensor::pose_t &pose_depth, const DepthSensor::intrinsics_t &intrinsics_depth,  const DepthSensor::disparity_params_t &disparity_params, const DepthSensor::disparity_type_t &disparity_type) {

    DepthSensor *depth_sensor = GetDepthSensor(pose_depth, intrinsics_depth,disparity_params,disparity_type);
    depth_sensor->Index = file.Sensors.size();
    file.Sensors.AddSensor(depth_sensor);

    if(!depth_sensor) {
        std::cout << "depth sensor not found..." << std::endl;
        return false;
    }

    std::string cam_name = "";
    std::string trajectory_name = "";
    std::string light_name = "";
    std::string time_name = "";
    std::string root_name = "";
    std::string directory_head = "";
    std::size_t found = dirname.find_last_of("/");
    if (found == dirname.length()-1) // the dirname ends with /
    {

    }
    else
    {
        cam_name = dirname.substr(found+1, 10);
        std::size_t found2 = dirname.rfind("/", found-1);
        std::size_t found3 = dirname.rfind("/", found2-1);
        std::size_t found4 = dirname.rfind("/", found3-1);
        std::size_t found5 = dirname.rfind("/", found4-1);
        trajectory_name = dirname.substr(found2+1,found-found2-1);
        light_name = dirname.substr(found3+1,found2-found3-1);
        time_name = dirname.substr(found4+1,found3-found4-1);
        root_name = dirname.substr(found5+1,found4-found5-1);
        directory_head = dirname.substr(0,found5);


        std::cout << "cam_name: " << cam_name  << std::endl;
        std::cout << "trajectory_name: " << trajectory_name  << std::endl;
        std::cout << "light_name: " << light_name  << std::endl;
        std::cout << "time_name: " << time_name  << std::endl;
        std::cout << "root_name: " << root_name  << std::endl;
        //std::cout << "directory_head: " << directory_head  << std::endl;
    }

    int frame_no = 0;

    std::string line;
    boost::smatch match;
    std::ifstream infile(directory_head + "/" + root_name + "/trajectories/" + trajectory_name + "/" + cam_name  + "/cam0_gt.visim");

    while (std::getline(infile, line)){
        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^([-0-9.e]+).*"))) {

            uint64_t time         = std::stoull(match[1]);
            double time_float   = time * std::pow(10, -9);
            uint32_t time_second  = std::floor(time_float);
            uint32_t time_nsecond = std::round((time_float - time_second)*std::pow(10, 9));

            ImageFileFrame *depth_frame = new ImageFileFrame();
            depth_frame->FrameSensor  = depth_sensor;
            depth_frame->Timestamp.S  = time_second;
            depth_frame->Timestamp.Ns = time_nsecond;
            depth_frame->ProcessCallback = undistort_frame;


            std::stringstream frame_name;
            frame_name << dirname << "/depth/"  <<  frame_no << ".png";
            depth_frame->Filename = frame_name.str();

            if(access(depth_frame->Filename.c_str(), F_OK) < 0) {
                printf("No depth image for frame (%s)\n", frame_name.str().c_str());
                perror("");
                return false;
            }

            file.AddFrame(depth_frame);
            frame_no++;

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }


    }

    std::cerr << "Checked Depth frames:" << frame_no << std::endl;
    return true;
}


bool loadICLRGBData(const std::string &dirname , SLAMFile &file, const Sensor::pose_t &pose, const CameraSensor::intrinsics_t &intrinsics) {

    CameraSensor *rgb_sensor = GetRGBSensor(pose, intrinsics);
    rgb_sensor->Index =file.Sensors.size();
    file.Sensors.AddSensor(rgb_sensor);

    if(!rgb_sensor) {
        std::cout << "rgb sensor not found..." << std::endl;
        return false;
    }


    std::string cam_name = "";
    std::string trajectory_name = "";
    std::string light_name = "";
    std::string time_name = "";
    std::string root_name = "";
    std::string directory_head = "";
    std::size_t found = dirname.find_last_of("/");
    if (found == dirname.length()-1) // the dirname ends with /
    {

    }
    else
    {
        cam_name = dirname.substr(found+1, 10);
        std::size_t found2 = dirname.rfind("/", found-1);
        std::size_t found3 = dirname.rfind("/", found2-1);
        std::size_t found4 = dirname.rfind("/", found3-1);
        std::size_t found5 = dirname.rfind("/", found4-1);
        trajectory_name = dirname.substr(found2+1,found-found2-1);
        light_name = dirname.substr(found3+1,found2-found3-1);
        time_name = dirname.substr(found4+1,found3-found4-1);
        root_name = dirname.substr(found5+1,found4-found5-1);
        directory_head = dirname.substr(0,found5);
    }
    int frame_no = 0;
    std::string line;
    boost::smatch match;
    std::ifstream infile(directory_head + "/" + root_name +  "/trajectories/" + trajectory_name + "/" + cam_name  + "/cam0_gt.visim");

    while (std::getline(infile, line)){
        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^([-0-9.e]+).*"))) {//else if (boost::regex_match(line,match,boost::regex("^([0-9]+).*"))) {

            uint64_t time         = std::stoull(match[1]);
            double time_float   = time * std::pow(10, -9);
            uint32_t time_second  = std::floor(time_float);
            uint32_t time_nsecond = std::round((time_float - time_second)*std::pow(10, 9));

            ImageFileFrame *rgb_frame = new ImageFileFrame();
            rgb_frame->FrameSensor  = rgb_sensor;
            rgb_frame->Timestamp.S  = time_second;
            rgb_frame->Timestamp.Ns = time_nsecond;

            std::stringstream frame_name;
            frame_name << dirname << "/rgb/"  <<  frame_no << ".png";
            rgb_frame->Filename = frame_name.str();

            if(access(rgb_frame->Filename.c_str(), F_OK) < 0) {
                printf("No RGB image for frame (%s)\n", frame_name.str().c_str());
                perror("");
                return false;
            }

            file.AddFrame(rgb_frame);
            frame_no++;

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    std::cerr << "Checked RGB frames:" << frame_no << std::endl;
    return true;
}

bool loadICLGreyData(const std::string &dirname , SLAMFile &file, const Sensor::pose_t &pose, const CameraSensor::intrinsics_t &intrinsics) {


    CameraSensor *grey_sensor = GetGreySensor(pose, intrinsics);
    grey_sensor->Index =file.Sensors.size();
    file.Sensors.AddSensor(grey_sensor);

    if(!grey_sensor) {
        std::cout << "grey sensor not found..." << std::endl;
        return false;
    }


    std::string cam_name = "";
    std::string trajectory_name = "";
    std::string light_name = "";
    std::string time_name = "";
    std::string root_name = "";
    std::string directory_head = "";
    std::size_t found = dirname.find_last_of("/");
    if (found == dirname.length()-1) // the dirname ends with /
    {

    }
    else
    {
        cam_name = dirname.substr(found+1, 10);
        std::size_t found2 = dirname.rfind("/", found-1);
        std::size_t found3 = dirname.rfind("/", found2-1);
        std::size_t found4 = dirname.rfind("/", found3-1);
        std::size_t found5 = dirname.rfind("/", found4-1);
        trajectory_name = dirname.substr(found2+1,found-found2-1);
        light_name = dirname.substr(found3+1,found2-found3-1);
        time_name = dirname.substr(found4+1,found3-found4-1);
        root_name = dirname.substr(found5+1,found4-found5-1);
        directory_head = dirname.substr(0,found5);
    }
    int frame_no = 0;
    std::string line;
    boost::smatch match;
    std::ifstream infile(directory_head + "/" + root_name +  "/trajectories/" + trajectory_name + "/" + cam_name  + "/cam0_gt.visim");

    while (std::getline(infile, line)){
        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^([-0-9.e]+).*"))) {//else if (boost::regex_match(line,match,boost::regex("^([0-9]+).*"))) {

            uint64_t time         = std::stoull(match[1]);
            double time_float   = time * std::pow(10, -9);
            uint32_t time_second  = std::floor(time_float);
            uint32_t time_nsecond = std::round((time_float - time_second)*std::pow(10, 9));


            ImageFileFrame *grey_frame = new ImageFileFrame();
            grey_frame->FrameSensor  = grey_sensor;
            grey_frame->Timestamp.S  = time_second;
            grey_frame->Timestamp.Ns = time_nsecond;

            std::stringstream frame_name;
            frame_name << dirname << "/rgb/"  <<  frame_no << ".png";
            grey_frame->Filename = frame_name.str();

            if(access(grey_frame->Filename.c_str(), F_OK) < 0) {
                printf("No GREY image for frame (%s)\n", frame_name.str().c_str());
                perror("");
                return false;
            }

            file.AddFrame(grey_frame);
            frame_no++;

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }
    }
    std::cerr << "Checked GREY frames:" << frame_no << std::endl;
    return true;
}


bool loadICLGroundTruthData(const std::string &dirname , SLAMFile &file) {

    GroundTruthSensor *gt_sensor = new GroundTruthSensor("GroundTruth");
    gt_sensor->Index = file.Sensors.size();
    gt_sensor->Description = "GroundTruthSensor";
    gt_sensor->Rate = 1;
    file.Sensors.AddSensor(gt_sensor);

    if(!gt_sensor) {
        std::cout << "gt sensor not found..." << std::endl;
        return false;
    } else {
        std::cout << "gt sensor created..." << std::endl;
    }

    std::string cam_name = "";
    std::string trajectory_name = "";
    std::string light_name = "";
    std::string time_name = "";
    std::string root_name = "";
    std::string directory_head = "";
    std::size_t found = dirname.find_last_of("/");
    if (found == dirname.length()-1) // the dirname ends with /
    {

    }
    else
    {
        cam_name = dirname.substr(found+1, 10);
        std::size_t found2 = dirname.rfind("/", found-1);
        std::size_t found3 = dirname.rfind("/", found2-1);
        std::size_t found4 = dirname.rfind("/", found3-1);
        std::size_t found5 = dirname.rfind("/", found4-1);
        trajectory_name = dirname.substr(found2+1,found-found2-1);
        light_name = dirname.substr(found3+1,found2-found3-1);
        time_name = dirname.substr(found4+1,found3-found4-1);
        root_name = dirname.substr(found5+1,found4-found5-1);
        directory_head = dirname.substr(0,found5);
    }
    std::string line;
    boost::smatch match;
    std::ifstream infile(directory_head + "/" + root_name +  "/trajectories/" + trajectory_name + "/" + cam_name  + "/cam0_gt.visim");

    int frame_no = 0;

    while (std::getline(infile, line)){
        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^([0-9]+),+([-0-9.e]+),+([-0-9.e]+),+([-0-9.e]+),+([-0-9.e]+),+([-0-9.e]+),+([-0-9.e]+),+([-0-9.e]+).*"))) {

            float tx =  std::stof(match[2]);
            float ty =  std::stof(match[3]);
            float tz =  std::stof(match[4]);

            float QW =  std::stof(match[5]);
            float QX =  std::stof(match[6]);
            float QY =  std::stof(match[7]);
            float QZ =  std::stof(match[8]);

            //std::cout << timestampNS << " " << tx << " " << ty << " " << tz << " " << QW << " " << QX << " " << QY << " " << QZ << std::endl;

            Eigen::Matrix3f rotationMat = Eigen::Quaternionf(QW,QZ,QX,QY).toRotationMatrix();

            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            //pose.block(0,0,3,3) = rotationMat;
            //pose.block(0,3,3,1) << tx , ty , tz;

            // convert to Povray here

            // modify translation/orientation
            pose(0,0) = rotationMat(0,0); pose(0,1) = rotationMat(0,1); pose(0,2) = rotationMat(0,2); pose(0,3) = tx;
            pose(1,0) = rotationMat(1,0); pose(1,1) = rotationMat(1,1); pose(1,2) = rotationMat(1,2); pose(1,3) = ty;
            pose(2,0) = rotationMat(2,0); pose(2,1) = rotationMat(2,1); pose(2,2) = rotationMat(2,2); pose(2,3) = tz;
            pose(3,0) = 0.0;              pose(3,1) = 0.0;              pose(3,2) = 0.0;              pose(3,3) = 1.0;

            Sensor::pose_t pose_temp = Eigen::Matrix4f::Identity();
            pose_temp << 1, 0 , 0 , 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;

            /*pose.block<2,3>(0,0) = -pose.block<2,3>(0,0);

             Sensor::pose_t pose_temp = Eigen::Matrix4f::Identity();
             pose_temp << 1, 0 , 0 , 0,
                          0, 0, -1, 0,
                          0, 1, 0, 0,
                          0, 0, 0, 1;*/

            pose = pose * pose_temp;

            uint64_t time         = std::stoull(match[1]);
            double time_float   = time * std::pow(10, -9);
            uint32_t time_second  = std::floor(time_float);
            uint32_t time_nsecond = std::round((time_float - time_second)*std::pow(10, 9));

            //std::cout<< std::fixed << time << " " << time_second << " " << time_nsecond << " " << (time_float - time_second)*std::pow(10, 9) << std::endl;



            SLAMInMemoryFrame *gt_frame = new SLAMInMemoryFrame();
            gt_frame->FrameSensor = gt_sensor;
            gt_frame->Data = malloc(gt_frame->GetSize());
            gt_frame->Timestamp.S  = time_second;
            gt_frame->Timestamp.Ns = time_nsecond;

            memcpy(gt_frame->Data,pose.data(),gt_frame->GetSize());
            //FillPose(frame_name.str(), *((GroundTruthSensor::pose_t*)gt_frame->Data));

            file.AddFrame(gt_frame);
            frame_no++;

        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }


    }

    std::cerr << "Checked GT frames:" << frame_no << std::endl;
    return true;
}


bool loadICLAccelerometerData(const std::string &dirname , SLAMFile &file) {

    AccelerometerSensor *accelerometer_sensor = new AccelerometerSensor("Accelerometer");
    accelerometer_sensor->Index = file.Sensors.size();
    accelerometer_sensor->Rate = 1;
    accelerometer_sensor->Description = "AccelerometerSensor";
    file.Sensors.AddSensor(accelerometer_sensor);

    if(!accelerometer_sensor) {
        std::cout << "accelerometer_sensor not found..." << std::endl;
        return false;
    }else {
        std::cout << "accelerometer_sensor created..." << std::endl;
    }


    std::string line;

    boost::smatch match;
    std::ifstream infile(dirname + "/" + "accelerometer.txt");

    while (std::getline(infile, line)){

        if (line.size() == 0) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
            continue;
        } else if (boost::regex_match(line,match,boost::regex("^([0-9]+)[.]([0-9]+)\\s+([-0-9.]+)\\s+([-0-9.]+)\\s+([-0-9.]+)$"))) {

            int timestampS = std::stoi(match[1]);
            int timestampNS = std::stoi(match[2]) *  std::pow ( 10, 9 - match[2].length());
            float ax =  std::stof(match[3]);
            float ay =  std::stof(match[4]);
            float az =  std::stof(match[5]);

            SLAMInMemoryFrame *accelerometer_frame = new SLAMInMemoryFrame();
            accelerometer_frame->FrameSensor = accelerometer_sensor;
            accelerometer_frame->Timestamp.S  = timestampS;
            accelerometer_frame->Timestamp.Ns = timestampNS;
            accelerometer_frame->Data = malloc(accelerometer_frame->GetSize());
            ((float*)accelerometer_frame->Data)[0] = ax;
            ((float*)accelerometer_frame->Data)[1] = ay;
            ((float*)accelerometer_frame->Data)[2] = az;

            file.AddFrame(accelerometer_frame);


        } else {
            std::cerr << "Unknown line:" << line << std::endl;
            return false;
        }


    }
    return true;
}


void frame_callback(int idx, int total) {
    printf("\r");

    // print progress bar
    printf("[");
    const int width = 50;
    float blocks = width * ((float)idx / total);

    for(int i = 0; i < blocks; ++i) {
        printf("#");
    }
    for(int i = blocks; i < width; ++i) {
        printf(" ");
    }
    printf("] ");

    printf("%u / %u", idx, total);
    fflush(stdout);
}
bool Serialise(const std::string &filename, SLAMFile &file) {
    return SLAMFile::Write(filename, file, frame_callback);
}




SLAMFile* ICLReader::GenerateSLAMFile () {

    if(!(grey || rgb || depth)) {
        std::cerr <<  "No sensors defined\n";
        return nullptr;
    }

    std::string dirname = input;

    if (!analyseIclFolder(dirname))	{
        std::cerr << "Invalid folder." << std::endl;
        return nullptr;
    }


    SLAMFile * slamfilep = new SLAMFile();
    SLAMFile & slamfile  = *slamfilep;


    /**
     * init SLAMFile
     */

    Sensor::pose_t pose = Eigen::Matrix4f::Identity();

    //////  Default are freiburg1

    CameraSensor::intrinsics_t intrinsics;
    DepthSensor::intrinsics_t intrinsics_depth;

    for (int i = 0; i < 4; i++) {
        intrinsics[i] = fr1_intrinsics_rgb[i];
        intrinsics_depth[i] = fr1_intrinsics_depth[i];
    }


    DepthSensor::disparity_params_t disparity_params =  {0.001,0.0};
    DepthSensor::disparity_type_t disparity_type = DepthSensor::affine_disparity;


    /**
     * load Depth
     */

    if(gt && depth && !loadICLDepthData(dirname, slamfile,pose,intrinsics_depth,disparity_params,disparity_type)) {
        std::cout << "Error while loading depth information." << std::endl;
        delete slamfilep;
        return nullptr;

    }


    /**
     * load Grey
     */

    if(grey && !loadICLGreyData(dirname, slamfile,pose,intrinsics)) {
        std::cout << "Error while loading Grey information." << std::endl;
        delete slamfilep;
        return nullptr;

    }


    /**
     * load RGB
     */

    if(gt && rgb && !loadICLRGBData(dirname, slamfile,pose,intrinsics)) {
        std::cout << "Error while loading RGB information." << std::endl;
        delete slamfilep;
        return nullptr;

    }


    /**
     * load GT
     */
    if(gt && !loadICLGroundTruthData(dirname, slamfile)) {
        std::cout << "Error while loading gt information." << std::endl;
        delete slamfilep;
        return nullptr;
    }


    /**
     * load Accelerometer: This one failed
     */
    if(accelerometer && !loadICLAccelerometerData(dirname, slamfile)) {
        std::cout << "Error while loading Accelerometer information." << std::endl;
        delete slamfilep;
        return nullptr;

    }


    return slamfilep;
}
constexpr CameraSensor::intrinsics_t ICLReader::fr1_intrinsics_rgb;
constexpr DepthSensor::intrinsics_t  ICLReader::fr1_intrinsics_depth;

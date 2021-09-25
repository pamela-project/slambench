/*

 Copyright (c) 2020 University of Manchester.
 Supported by the RAIN Hub, which is funded by the Industrial Strategy Challenge Fund,
 part of the UK government’s modern Industrial Strategy. The fund is
 delivered by UK Research and Innovation and managed by EPSRC [EP/R026084/1].
 This code is licensed under the MIT License.

 */

#include "include/VolumeDeform.h"
#include "include/utils/dataset_utils.h"
#include "io/sensor/sensor_builder.h"

#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/sensor/GroundTruthSensor.h>
#include <Eigen/Eigen>

#include <iostream>

using namespace slambench::io;

std::string GetFileName(const std::string& dirname, size_t frame_idx, bool is_depth_img)
{
    //Construct the file_name
    char frame_idx_str[20];
    sprintf(frame_idx_str, "%06d", static_cast<int>(frame_idx));
    std::string file_name = "frame-";
    file_name += std::string(frame_idx_str);
    if (is_depth_img) {
        file_name += ".depth";
    }
    else {
        file_name += ".color";
    }
    file_name += ".png";

    return dirname + "/" + file_name;
}

bool VolumeDeformReader::GetFrame(const std::string &dirname,
                                  int frame_no,
                                  SLAMFile &file,
                                  std::ifstream& infile,
                                  CameraSensor* rgb_sensor,
                                  CameraSensor* grey_sensor,
                                  DepthSensor* depth_sensor,
                                  GroundTruthSensor* gt_sensor) {

    double frame_time = 1.0 / image_params.rate;

    uint64_t total_ns = frame_time * frame_no * 1000000000;
    slambench::TimeStamp ts;
    ts.S = total_ns / 1000000000;
    ts.Ns = total_ns % 1000000000;

    if (rgb_sensor) {
        auto rgb_frame = new ImageFileFrame();
        rgb_frame->FrameSensor = rgb_sensor;
        rgb_frame->Timestamp = ts;;
        rgb_frame->filename = GetFileName(dirname, frame_no, false);

        if (access(rgb_frame->filename.c_str(), F_OK) < 0) {
            printf("No RGB image for frame %d (%s)\n", frame_no, rgb_frame->filename.c_str());
            return false;
        }

        file.AddFrame(rgb_frame);
    }
    if (grey_sensor) {
        auto grey_frame = new ImageFileFrame();
        grey_frame->FrameSensor = grey_sensor;
        grey_frame->Timestamp = ts;
        grey_frame->filename = GetFileName(dirname, frame_no, false);

        if (access(grey_frame->filename.c_str(), F_OK) < 0) {
            printf("No Grey image for frame %d (%s)\n", frame_no, grey_frame->filename.c_str());
            perror("");
            return false;
        }

        file.AddFrame(grey_frame);
    }

    if (depth_sensor) {
        auto depth_frame = new ImageFileFrame();
        depth_frame->filename = GetFileName(dirname, frame_no, true);
        depth_frame->FrameSensor = depth_sensor;
        depth_frame->Timestamp = ts;

        if (access(depth_frame->filename.c_str(), F_OK) < 0) {
            printf("No depth image for frame %d (%s)\n", frame_no, depth_frame->filename.c_str());
            perror("");
            return false;
        }

        file.AddFrame(depth_frame);
    }

    if (gt_sensor) {
        std::vector<float> myNumbers;
        float number;
        std::string line;
        std::getline(infile, line);
        if(line.empty())
            return false;
        std::stringstream iss( line );
        while ( iss >> number )
            myNumbers.push_back( number );
        auto pose = Eigen::Map<slambench::io::Sensor::pose_t>(myNumbers.data()).transpose(); // FIXME: why doesn't using ColMajor instead fix this?
        auto gt_frame = new SLAMInMemoryFrame();
        gt_frame->FrameSensor = gt_sensor;
        gt_frame->Timestamp = ts;
        gt_frame->Data = malloc(sizeof(slambench::io::Sensor::pose_t));

        memcpy(gt_frame->Data, pose.data(), gt_frame->GetSize());
        file.AddFrame(gt_frame);
    }

    return true;
}

SLAMFile *VolumeDeformReader::GenerateSLAMFile() {

    if (!(grey || rgb || depth)) {
        std::cerr << "No sensors defined\n";
        return nullptr;
    }

    const std::vector<std::string> requirements = {"data",
                                                   "canonical",
                                                   "reconstruction",
                                                   "poses.txt"};

    if (!checkRequirements(input, requirements)) {
        std::cerr << "Invalid folder." << std::endl;
        return nullptr;
    }

    auto slamfile_ptr = new SLAMFile();
    auto &slamfile = *slamfile_ptr;
    CameraSensor *rgb_sensor, *grey_sensor;
    DepthSensor *depth_sensor;
    GroundTruthSensor *gt_sensor;
    // load Depth
    if (depth) {
        depth_sensor = DepthSensorBuilder()
                .rate(image_params.rate)
                .size(image_params.width, image_params.height)
                .intrinsics(intrinsics)
                .build();

        slamfile.Sensors.AddSensor(depth_sensor);
    }

    // load Grey
    if (grey) {
        grey_sensor = GreySensorBuilder()
                .rate(image_params.rate)
                .size(image_params.width, image_params.height)
                .intrinsics(intrinsics)
                .build();

        slamfile.Sensors.AddSensor(grey_sensor);
    }

    // load RGB
    if (rgb) {
        rgb_sensor = RGBSensorBuilder()
                .rate(image_params.rate)
                .size(image_params.width, image_params.height)
                .intrinsics(intrinsics)
                .build();

        slamfile.Sensors.AddSensor(rgb_sensor);
    }

    // load GT
    if (gt) {
        gt_sensor = GTSensorBuilder().build();
        slamfile.Sensors.AddSensor(gt_sensor);
    }
    auto gt_file = input + "/poses.txt";
    std::ifstream istream;
    if(access(gt_file.c_str(), F_OK) >= 0)
        istream = std::ifstream(gt_file);

    int frame_no = 0;
    while (GetFrame(input+"/data", frame_no, slamfile, istream, rgb_sensor, grey_sensor, depth_sensor, gt_sensor)) {
        frame_no++;
    }
    if(istream.is_open())
        istream.close();

    return slamfile_ptr;
}

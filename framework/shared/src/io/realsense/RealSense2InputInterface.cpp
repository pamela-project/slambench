/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include <io/realsense/RealSense2InputInterface.h>
#include <io/sensor/sensor_builder.h>
#include <io/sensor/SensorCollection.h>
#include <io/realsense/RealSense2FrameStream.h>
#include <librealsense2/rs.hpp>
#include <iostream>

using namespace slambench::io;
using namespace slambench::io::realsense;

RealSense2InputInterface::RealSense2InputInterface() : stream_(nullptr), sensors_ready_(false) {

    try {
        std::cout << "Querying Realsense device info..." << std::endl;

        // Create librealsense context and check device is connected
        rs2::context ctx;
        auto devs = ctx.query_devices();
        int device_num = devs.size();
        assert(device_num && "Device not found!");
        std::cout << "Device number: " << device_num << std::endl;

        // Query the info of first device
        // Device serial number (different for each device, can be used for searching device when having mutiple devices)
        std::cout << "Serial number: " << devs.front().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
        BuildStream();
    }
    catch(const rs2::error &e){
        // Capture device exception
        std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << "\nExiting!" << std::endl;
        exit(EXIT_FAILURE);
    }
}

FrameStream& RealSense2InputInterface::GetFrames() {
    if(stream_ == nullptr) BuildStream();

    return *stream_;
}

SensorCollection& RealSense2InputInterface::GetSensors() {
    BuildStream();
    return sensors_;
}

void RealSense2InputInterface::BuildRGBSensor(const rs2::stream_profile& color_profile) {
    rs2::video_stream_profile cvsprofile(color_profile);
    rs2_intrinsics color_intrinsics = cvsprofile.get_intrinsics();

    slambench::io::CameraSensor::intrinsics_t intrinsics;
    intrinsics[0] = color_intrinsics.fx / color_intrinsics.width;
    intrinsics[1] = color_intrinsics.fy / color_intrinsics.height;
    intrinsics[2] = color_intrinsics.ppx / color_intrinsics.width;
    intrinsics[3] = color_intrinsics.ppy / color_intrinsics.height;


    slambench::io::CameraSensor::distortion_coefficients_t distortion;
    for(size_t i = 0; i < 5; i++)
        distortion[i] = color_intrinsics.coeffs[i];

    slambench::io::CameraSensor::distortion_type_t distortion_type;
    if (color_intrinsics.model == rs2_distortion::RS2_DISTORTION_NONE) {
        distortion_type = slambench::io::CameraSensor::NoDistortion;
    } else if (color_intrinsics.model == rs2_distortion::RS2_DISTORTION_BROWN_CONRADY || color_intrinsics.model == rs2_distortion::RS2_DISTORTION_INVERSE_BROWN_CONRADY) {
        distortion_type = slambench::io::CameraSensor::RadialTangential;
    } else if (color_intrinsics.model == rs2_distortion::RS2_DISTORTION_KANNALA_BRANDT4) {
        distortion_type = slambench::io::CameraSensor::KannalaBrandt;
    } else {
        std::cerr<<"Unsupported distortion type!"<<std::endl;
        exit(EXIT_FAILURE);
    }
    auto sensor = CameraSensorBuilder()
            .name(color_profile.stream_name())
            .description("Intel Realsense RGB")
            .rate(color_profile.fps())
            .size(color_intrinsics.width, color_intrinsics.height)
            .intrinsics(intrinsics)
            .distortion(distortion_type, distortion)
            .frameFormat(frameformat::Raster)
            .pixelFormat(pixelformat::RGB_III_888)
            .index(sensors_.size())
            .build();
    sensors_.AddSensor(sensor);
}

void RealSense2InputInterface::BuildAccelerometerGyroSensor(const rs2::stream_profile& motion_profile) {

    Eigen::Matrix4f pose;
    rs2::motion_stream_profile mprofile(motion_profile);
    auto intrinsics = mprofile.get_motion_intrinsics();

    // TODO: get pose to RGB camera motion.get_profile().get_extrinsics_to()
    // If casting succeeded and the arrived frame is from gyro stream
    if (mprofile.stream_type() == RS2_STREAM_GYRO && mprofile.format() == RS2_FORMAT_MOTION_XYZ32F)
    {
        auto gyro_sensor = GyroSensorBuilder()
                .name(mprofile.stream_name())
                .rate(mprofile.fps())
                .description("Realsense Gyro")
                .index(sensors_.size())
                .pose(pose)
                .build();
        //rs2_vector gyro_data = motion.get_profile().stream_name();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                gyro_sensor->Intrinsic[i] = gyro_sensor->Intrinsic[i*4+j] = intrinsics.data[i][j];
            }
        }
        for (int i = 0; i < 3; i++) {
            gyro_sensor->NoiseVariances[i] = intrinsics.noise_variances[i];
            gyro_sensor->BiasVariances[i] = intrinsics.bias_variances[i];
        }
        sensors_.AddSensor(gyro_sensor);
    }
    // If casting succeeded and the arrived frame is from accelerometer stream
    if (mprofile.stream_type() == RS2_STREAM_ACCEL && mprofile.format() == RS2_FORMAT_MOTION_XYZ32F)
    {
        auto accelerometer_sensor = AccSensorBuilder()
                .name(mprofile.stream_name())
                .rate(mprofile.fps())
                .description("Realsense Accelerometer")
                .index(sensors_.size())
                .pose(pose)
                .build();

        // Get accelerometer measures
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                accelerometer_sensor->Intrinsic[i*4+j] = intrinsics.data[i][j];
            }
        }

        for (int i = 0; i < 3; i++) {
            accelerometer_sensor->NoiseVariances[i] = intrinsics.noise_variances[i];
            accelerometer_sensor->BiasVariances[i] = intrinsics.bias_variances[i];
        }
        sensors_.AddSensor(accelerometer_sensor);
    }
}

void RealSense2InputInterface::BuildDepthSensor(const rs2::stream_profile& depth_profile) {

    rs2::video_stream_profile dvsprofile(depth_profile);
    rs2_intrinsics depth_intrinsics = dvsprofile.get_intrinsics();
    slambench::io::CameraSensor::intrinsics_t intrinsics;
    intrinsics[0] = depth_intrinsics.fx / depth_intrinsics.width;
    intrinsics[1] = depth_intrinsics.fy / depth_intrinsics.height;
    intrinsics[2] = depth_intrinsics.ppx / depth_intrinsics.width;
    intrinsics[3] = depth_intrinsics.ppy / depth_intrinsics.height;

    slambench::io::CameraSensor::distortion_coefficients_t distortion;
    for(size_t i = 0; i < 5; i++)
        distortion[i] = depth_intrinsics.coeffs[i];

    slambench::io::CameraSensor::distortion_type_t distortion_type;
    if (depth_intrinsics.model == rs2_distortion::RS2_DISTORTION_NONE) {
        distortion_type = slambench::io::CameraSensor::NoDistortion;
    } else if (depth_intrinsics.model == rs2_distortion::RS2_DISTORTION_BROWN_CONRADY || depth_intrinsics.model == rs2_distortion::RS2_DISTORTION_INVERSE_BROWN_CONRADY) {
        distortion_type = slambench::io::CameraSensor::RadialTangential;
    } else if (depth_intrinsics.model == rs2_distortion::RS2_DISTORTION_KANNALA_BRANDT4) {
        distortion_type = slambench::io::CameraSensor::KannalaBrandt;
    } else {
        std::cerr<<"Unsupported distortion type!"<<std::endl;
        exit(EXIT_FAILURE);
    }
    auto depth_sensor = DepthSensorBuilder()
            .name(depth_profile.stream_name())
            .description("Intel Realsense RGB")
            .size(dvsprofile.get_intrinsics().width, dvsprofile.get_intrinsics().height)
            .intrinsics(intrinsics)
            .frameFormat(frameformat::Raster)
            .pixelFormat(pixelformat::D_I_16)
            .rate(depth_profile.fps())
            .distortion(distortion_type, distortion)
            .index(sensors_.size())
            //.disparity(disparity_type, disparity_params)
            .build();

    sensors_.AddSensor(depth_sensor);
}

void RealSense2InputInterface::BuildStream() {
    if(sensors_ready_) return;
    // FIXME: config parameters should come from user and also have a default

    rs2::config cfg;
    pipe_ = rs2::pipeline();

    // Config color stream: 640*480, frame format: BGR (CV_8UC3), FPS: 30
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGB8, 30);

    // Config depth stream: 640*480, frame format: Z16 (CV_16U), FPS: 30
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    // Accelerometer and gyro
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    auto profile = pipe_.start(cfg);
    auto color_stream = profile.get_stream(RS2_STREAM_COLOR);
    BuildRGBSensor(color_stream);

    auto depth_stream = profile.get_stream(RS2_STREAM_DEPTH);
    BuildDepthSensor(depth_stream);

    auto acc_stream = profile.get_stream(RS2_STREAM_ACCEL);
    BuildAccelerometerGyroSensor(acc_stream);

    auto gyro_stream = profile.get_stream(RS2_STREAM_GYRO);
    BuildAccelerometerGyroSensor(gyro_stream);

    stream_ = new RealSense2FrameStream(pipe_);
    bool depth_found = false, rgb_found = false, gyro_found = false, acc_found = false;
    for(auto *sensor : sensors_) {
        if(sensor->GetType() == slambench::io::CameraSensor::kCameraType) {
            rgb_found = true;
            stream_->ActivateSensor(RS2_STREAM_COLOR, (CameraSensor*)sensor);
        } else if (sensor->GetType() == slambench::io::DepthSensor::kDepthType) {
            depth_found = true;
            stream_->ActivateSensor(RS2_STREAM_DEPTH, (DepthSensor*)sensor);
        } else if (sensor->GetType() == slambench::io::AccelerometerSensor::kAccType) {
            acc_found = true;
            stream_->ActivateSensor(RS2_STREAM_ACCEL, (AccelerometerSensor*)sensor);
        } else if (sensor->GetType() == slambench::io::GyroSensor::kGyroType) {
            gyro_found = true;
            stream_->ActivateSensor(RS2_STREAM_GYRO, (GyroSensor*)sensor);
        }
    }
    assert(rgb_found and depth_found);

    if(acc_found && gyro_found)
        std::cout<<"Motion data available"<<std::endl;
    else
        std::cerr<<"Motion data NOT available"<<std::endl;

}

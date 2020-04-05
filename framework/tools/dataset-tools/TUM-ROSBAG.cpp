/*

 Copyright (c) 2017-2020 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 The development of the interface with ROS datasets (rosbags) is supported
 by the RAIN Hub, which is funded by the Industrial Strategy Challenge Fund,
 part of the UK government’s modern Industrial Strategy. The fund is
 delivered by UK Research and Innovation and managed by EPSRC [EP/R026084/1].

 This code is licensed under the MIT License.

 */


#include <iostream>

#include <Eigen/Eigen>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/AccelerometerSensor.h>

#include "../dataset-tools/include/TUM-ROSBAG.h"


using namespace slambench::io ;


bool loadTUMROSDepthData(const std::string &dirname, const std::string &bagname,
        SLAMFile &file, const float depthMapFactor, const Sensor::pose_t &pose,
        const DepthSensor::intrinsics_t &intrinsics,
        const CameraSensor::distortion_coefficients_t &distortion,
        const DepthSensor::disparity_params_t &disparity_params,
        const DepthSensor::disparity_type_t &disparity_type) {

    // populate sensor data
    auto *depth_sensor = new DepthSensor("Depth");
    if (depth_sensor == nullptr) {
        std::cerr << "error: depth sensor not found" << std::endl;
        return false;
    }
    std::cout << "depth sensor created" << std::endl;

    depth_sensor->Index = 0;
    depth_sensor->Width = 640;
    depth_sensor->Height = 480;
    depth_sensor->FrameFormat = frameformat::Raster;
    depth_sensor->PixelFormat = pixelformat::D_I_16;
    depth_sensor->DisparityType = disparity_type;
    depth_sensor->Description = "Depth";
    depth_sensor->CopyPose(pose);
    depth_sensor->CopyIntrinsics(intrinsics);
    depth_sensor->CopyDisparityParams(disparity_params);
    depth_sensor->DistortionType = slambench::io::CameraSensor::RadialTangential;
    depth_sensor->CopyRadialTangentialDistortion(distortion);
    depth_sensor->Index = file.Sensors.size();
    depth_sensor->Rate = 30.0;
    file.Sensors.AddSensor(depth_sensor);

    // initialised to avoid warnings!
    uint32_t sec  = 0;
    uint32_t nsec = 0;

    // create directory for depth images
    std::string depthdir = dirname + "/depth/";
    if (!boost::filesystem::exists(depthdir)) {
        if (!boost::filesystem::create_directory(depthdir)) {
            std::cerr << "error creating depth directory: " <<
                    depthdir << std::endl;
            return false;
        }
    }

    // open rosbag file
    rosbag::Bag bag;
    try {
        bag.open(bagname,rosbag::bagmode::Read);
    }
    catch (...) {
        std::cerr << "error opening depth rosbag: " << bagname << std::endl;
        return false;
    }

    // create query to fetch depth topic messages
    rosbag::View view(bag, rosbag::TopicQuery("/camera/depth/image"));

    // produce png image for every depth message
    for (const auto& msg : view) {
        sensor_msgs::Image::ConstPtr imgi = msg.instantiate<sensor_msgs::Image>();
        if (imgi == nullptr) {
            continue;
        }

        cv::Mat imgo = cv::Mat(imgi->height, imgi->width, CV_32FC1,
                const_cast<uchar*>(&imgi->data[0]), imgi->step);
        cv::Mat image = cv::Mat(imgi->height, imgi->width, CV_16UC1);
        for (uint r = 0; r < imgi->height; r++) {
            for (uint c = 0; c < imgi->width; c++) {
                float dist = imgo.at<float>(r * imgi->width + c);
                auto dist16 = (unsigned short) (depthMapFactor * dist);
                image.at<short>(r, c) = dist16;
            }
        }

        // record time stamp with adjusted precision
        // TUM RGB-D datasets use 6-digit nanosec timestamps
        sec = imgi->header.stamp.sec;
        nsec = (imgi->header.stamp.nsec + 500)/1000;
        if (nsec >= 1000000) {
            sec++;
            nsec = 0;
        }

        std::stringstream frame_name;
        frame_name << depthdir;
        frame_name << sec << ".";
        frame_name << std::setw(6) << std::setfill('0') << nsec << ".png";
        if (!cv::imwrite(frame_name.str(), image)) {
            std::cerr << "error writing depth image: " <<
                    frame_name.str() << std::endl;
            return false;
        }

        // update slambench file with new frame
        auto *depth_frame = new ImageFileFrame();
        if (depth_frame == nullptr) {
            std::cerr << "error creating depth image frame" << std::endl;
            return false;
        }

        depth_frame->FrameSensor  = depth_sensor;
        depth_frame->Timestamp.S  = sec;
        depth_frame->Timestamp.Ns = nsec;
        depth_frame->Filename     = frame_name.str();
        file.AddFrame(depth_frame);
    }

    bag.close();

    return true;
}


bool loadTUMROSRGBGreyData(bool doRGB, bool doGrey, const std::string &dirname,
        const std::string &bagname, SLAMFile &file, const Sensor::pose_t &pose,
        const CameraSensor::intrinsics_t &intrinsics,
        const CameraSensor::distortion_coefficients_t &distortion) {

    auto *rgb_sensor = new CameraSensor("RGB", CameraSensor::kCameraType);
    if (rgb_sensor == nullptr) {
        std::cerr << "error: rgb sensor not found" << std::endl;
        return false;
    }
    std::cout << "rgb sensor created" << std::endl;

    auto *grey_sensor = new CameraSensor("Grey", CameraSensor::kCameraType);
    if (grey_sensor == nullptr) {
        std::cerr << "error: grey sensor not found" << std::endl;
        return false;
    }
    std::cout << "grey sensor created" << std::endl;

    // populate rgb sensor data
    if (doRGB) {
        rgb_sensor->Index = 0;
        rgb_sensor->Width = 640;
        rgb_sensor->Height = 480;
        rgb_sensor->FrameFormat = frameformat::Raster;
        rgb_sensor->PixelFormat = pixelformat::RGB_III_888;
        rgb_sensor->Description = "RGB";
        rgb_sensor->CopyPose(pose);
        rgb_sensor->CopyIntrinsics(intrinsics);
        rgb_sensor->DistortionType = slambench::io::CameraSensor::RadialTangential;
        rgb_sensor->CopyRadialTangentialDistortion(distortion);
        rgb_sensor->Index = file.Sensors.size();
        rgb_sensor->Rate = 30.0;
        file.Sensors.AddSensor(rgb_sensor);
    }

    // populate grey sensor data
    if (doGrey) {
        grey_sensor->Index = 0;
        grey_sensor->Width = 640;
        grey_sensor->Height = 480;
        grey_sensor->FrameFormat = frameformat::Raster;
        grey_sensor->PixelFormat = pixelformat::G_I_8;
        grey_sensor->Description = "Grey";
        grey_sensor->CopyPose(pose);
        grey_sensor->CopyIntrinsics(intrinsics);
        grey_sensor->CopyRadialTangentialDistortion(distortion);
        grey_sensor->DistortionType = slambench::io::CameraSensor::RadialTangential;
        grey_sensor->Index = file.Sensors.size();
        grey_sensor->Rate = 30.0;
        file.Sensors.AddSensor(grey_sensor);
    }

    // initialised to avoid warnings!
    uint32_t sec  = 0;
    uint32_t nsec = 0;

    // create directory for rgb images
    std::string rgbdir = dirname + "/rgb/";
    if (!boost::filesystem::exists(rgbdir)) {
        if (!boost::filesystem::create_directory(rgbdir)) {
            std::cerr << "error creating rgb directory: " << rgbdir << std::endl;
            return false;
        }
    }

    // open rosbag file
    rosbag::Bag bag;
    try {
        bag.open(bagname, rosbag::bagmode::Read);
    }
    catch (...) {
        std::cerr << "error opening rgb rosbag: " << bagname << std::endl;
        return false;
    }

    // create query to fetch rgb images topic messages
    // NOTE: TUM rosbag files do not contain grey images - use rgb ones!
    rosbag::View view(bag, rosbag::TopicQuery("/camera/rgb/image_color"));

    // produce png image for every rgb message
    for (const auto& msg : view) {
        sensor_msgs::Image::ConstPtr imgi = msg.instantiate<sensor_msgs::Image>();
        if (imgi == nullptr) {
            continue;
        }

        cv::Mat imgo = cv::Mat(imgi->height, imgi->width, CV_8UC3,
                               const_cast<uchar *>(&imgi->data[0]), imgi->step);
        cv::Mat image = cv::Mat(imgi->height, imgi->width, CV_8UC3);
        for (uint r = 0; r < imgi->height; r++) {
            for (uint c = 0; c < imgi->width; c++) {
                // NOTE: OpenCV defaults to BGR8 encoding!
                image.at<cv::Vec3b>(r, c)[0] = imgo.at<cv::Vec3b>(r, c)[2];
                image.at<cv::Vec3b>(r, c)[1] = imgo.at<cv::Vec3b>(r, c)[1];
                image.at<cv::Vec3b>(r, c)[2] = imgo.at<cv::Vec3b>(r, c)[0];
            }
        }

        // record time stamp with adjusted precision
        // TUM RGB-D datasets use 6-digit nanosec image timestamps
        sec = imgi->header.stamp.sec;
        nsec = (imgi->header.stamp.nsec + 500)/1000;
        if (nsec >= 1000000) {
            sec++;
            nsec = 0;
        }

        std::stringstream frame_name;
        frame_name << rgbdir;
        frame_name << sec << ".";
        frame_name << std::setw(6) << std::setfill('0') << nsec << ".png";
        if (!cv::imwrite(frame_name.str(), image)) {
            std::cerr << "error writing rgb image: " <<
                    frame_name.str() << std::endl;
            return false;
        }

        // update slambench file with new rgb frame
        if (doRGB) {
            auto *rgb_frame = new ImageFileFrame();
            if (rgb_frame == nullptr) {
                std::cerr << "error creating rgb image frame" << std::endl;
                return false;
            }

            rgb_frame->FrameSensor = rgb_sensor;
            rgb_frame->Timestamp.S = sec;
            rgb_frame->Timestamp.Ns = nsec;
            rgb_frame->Filename = frame_name.str();
            file.AddFrame(rgb_frame);
        }

        // update slambench file with new grey frame
        if (doGrey) {
            auto *grey_frame = new ImageFileFrame();
            if (grey_frame == nullptr) {
                std::cerr << "error creating grey image frame" << std::endl;
                return false;
            }

            grey_frame->FrameSensor = grey_sensor;
            grey_frame->Timestamp.S = sec;
            grey_frame->Timestamp.Ns = nsec;
            grey_frame->Filename = frame_name.str();
            file.AddFrame(grey_frame);
        }
    }

    bag.close();

    return true;
}


bool loadTUMROSGroundTruthData(const std::string &bagname, SLAMFile &file) {

    auto *gt_sensor = new GroundTruthSensor("GroundTruth");
    if (gt_sensor == nullptr) {
        std::cerr << "error: gt sensor not found" << std::endl;
        return false;
    }
    std::cout << "gt sensor created" << std::endl;

    gt_sensor->Index = file.Sensors.size();
    gt_sensor->Description = "GroundTruthSensor";
    file.Sensors.AddSensor(gt_sensor);

    std::string world_str ("/world");
    std::string kinect_str ("/kinect");
    std::string camera_str ("/openni_camera");
    std::string rgb_str ("/openni_rgb_frame");
    std::string opt_str ("/openni_rgb_optical_frame");

    // initialised to avoid warnings!
    tf::Transform w_k_trans = tf::Transform::getIdentity();
    tf::Transform k_c_trans = tf::Transform::getIdentity();
    tf::Transform c_r_trans = tf::Transform::getIdentity();
    tf::Transform r_o_trans = tf::Transform::getIdentity();

    bool w_k_new = false;
    bool k_c_rdy = false;
    bool c_r_rdy = false;
    bool r_o_rdy = false;
    bool all_rdy = false;

    // initialised to avoid warnings!
    uint32_t sec  = 0;
    uint32_t nsec = 0;

    // open rosbag
    rosbag::Bag bag;
    try {
        bag.open(bagname,rosbag::bagmode::Read);
    }
    catch (...) {
        std::cerr << "error opening gt rosbag: " << bagname << std::endl;
        return false;
    }

    // create query to fetch ground truth topic messages
    rosbag::View view(bag, rosbag::TopicQuery("/tf"));

    for (const auto& msg : view) {
        tf::tfMessage::ConstPtr msgi = msg.instantiate<tf::tfMessage>();
        if (msgi == nullptr) {
            continue;
        }

        for (const auto& msgii : msgi->transforms) {
            if (!r_o_rdy && msgii.child_frame_id == opt_str) {
                // record the /openni_rgb_frame to /openni_rgb_optical_frame
                // transformation only once
                if ((msgii.header.frame_id == rgb_str)) {
                    r_o_rdy = true;
                    all_rdy = r_o_rdy && c_r_rdy && k_c_rdy;
                    tf::transformMsgToTF(msgii.transform, r_o_trans);
                }
            } else if (!c_r_rdy && msgii.child_frame_id == rgb_str) {
                // record the /openni_camera to /openni_rgb_frame
                // transformation only once
                if ((msgii.header.frame_id == camera_str)) {
                    c_r_rdy = true;
                    all_rdy = r_o_rdy && c_r_rdy && k_c_rdy;
                    tf::transformMsgToTF(msgii.transform, c_r_trans);
                }
            } else if (!k_c_rdy && msgii.child_frame_id == camera_str) {
                // record the /kinect to /openni_camera
                // transformation only once
                if ((msgii.header.frame_id == kinect_str)) {
                    k_c_rdy = true;
                    all_rdy = r_o_rdy && c_r_rdy && k_c_rdy;
                    tf::transformMsgToTF(msgii.transform, k_c_trans);
                }
            } else if (msgii.child_frame_id == kinect_str) {
                // track continuously the /world to /kinect transformation
                if (msgii.header.frame_id == world_str) {
                    w_k_new = true;
                    // record time stamp with adjusted precision
                    sec = msgii.header.stamp.sec;

                    // record time stamp with adjusted precision
                    // TUM RGB-D datasets use 4-digit gt nanosec timestamps
                    nsec = (msgii.header.stamp.nsec + 50000)/100000;
                    if (nsec >= 10000) {
                        sec++;
                        nsec = 0;
                    }
                    tf::transformMsgToTF(msgii.transform, w_k_trans);
                }
            }
            if (all_rdy && w_k_new) {
                w_k_new = false;
                Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
                tf::Transform w_o_trans = w_k_trans * k_c_trans *
                        c_r_trans * r_o_trans;

                tf::Vector3 tr = w_o_trans.getOrigin();
                tf::Matrix3x3 rt = w_o_trans.getBasis();

                // NOTE: implicit float64 (double) to float casts
                pose.block(0, 0, 3, 3) <<
                        rt[0][0], rt[0][1], rt[0][2],
                        rt[1][0], rt[1][1], rt[1][2],
                        rt[2][0], rt[2][1], rt[2][2];
                pose.block(0, 3, 3, 1) <<
                        tr.x(), tr.y(), tr.z();

                auto *gt_frame = new SLAMInMemoryFrame();
                if (gt_frame == nullptr) {
                    std::cerr << "error creating gt in-memory frame" << std::endl;
                    return false;
                }

                gt_frame->FrameSensor = gt_sensor;
                gt_frame->Timestamp.S = sec;
                gt_frame->Timestamp.Ns = nsec;
                gt_frame->Data = malloc(gt_frame->GetSize());
                if (gt_frame->Data == nullptr) {
                    std::cerr << "error allocating memory for gt frame" << std::endl;
                    return false;
                }

                memcpy(gt_frame->Data, pose.data(), gt_frame->GetSize());

                file.AddFrame(gt_frame);

                break;
            }
        }
    }

    bag.close();

    return true;
}


bool loadTUMROSAccelerometerData(const std::string &bagname, SLAMFile &file) {

    auto *accelerometer_sensor = new AccelerometerSensor("Accelerometer");
    if (accelerometer_sensor == nullptr) {
        std::cerr << "error: accelerometer_sensor not found" << std::endl;
        return false;
    }
    std::cout << "accelerometer sensor created" << std::endl;

    accelerometer_sensor->Index = file.Sensors.size();
    accelerometer_sensor->Description = "AccelerometerSensor";
    file.Sensors.AddSensor(accelerometer_sensor);

    // initialised to avoid warnings!
    uint32_t sec = 0;
    uint32_t nsec = 0;

    // open rosbag
    rosbag::Bag bag;
    try {
        bag.open(bagname,rosbag::bagmode::Read);
    }
    catch (...) {
        std::cerr << "error opening accelerometer rosbag: " <<
                bagname << std::endl;
        return false;
    }

    // create query to fetch ground truth topic messages
    rosbag::View view(bag, rosbag::TopicQuery("/imu"));

    for (const auto& msg : view) {
        sensor_msgs::Imu::ConstPtr msgi = msg.instantiate<sensor_msgs::Imu>();
        if (msgi == nullptr) {
            continue;
        }

        // record time stamp with adjusted precision
        // TUM RGB-D datasets use 6-digit accelerometer nanosec timestamps
        sec = msgi->header.stamp.sec;
        nsec = (msgi->header.stamp.nsec + 500) / 1000;
        if (nsec >= 1000000) {
            sec++;
            nsec = 0;
        }

        geometry_msgs::Vector3 lacc = msgi->linear_acceleration;

        auto *accelerometer_frame = new SLAMInMemoryFrame();
        if (accelerometer_frame == nullptr) {
            std::cerr << "error creating acc in-memory frame" << std::endl;
            return false;
        }

        accelerometer_frame->FrameSensor = accelerometer_sensor;
        accelerometer_frame->Timestamp.S = sec;
        accelerometer_frame->Timestamp.Ns = nsec;
        accelerometer_frame->Data = malloc(accelerometer_frame->GetSize());
        if (accelerometer_frame->Data == nullptr) {
            std::cerr << "error allocating memory for acc frame" << std::endl;
            return false;
        }

        // NOTE: float64 (double) to float casts
        ((float *) accelerometer_frame->Data)[0] = (float) lacc.x;
        ((float *) accelerometer_frame->Data)[1] = (float) lacc.y;
        ((float *) accelerometer_frame->Data)[2] = (float) lacc.z;

        file.AddFrame(accelerometer_frame);
    }

    bag.close();

    return true;
}


SLAMFile* TUMROSReader::GenerateSLAMFile () {

    if (!(grey || rgb || depth)) {
        std::cerr <<  "error: no sensors defined" << std::endl;
        return nullptr;
    }

    std::string dirname = input;

    // rosbag file
    auto pos = dirname.rfind('/');
    if (pos == std::string::npos) {
        pos = 0;
    }
    std::string bagname = dirname + "/../../" + dirname.substr(pos) + ".bag";

    auto * slamfilep = new SLAMFile();
    if (slamfilep == nullptr) {
        std::cerr << "error creating slamfile handle" << std::endl;
        return nullptr;
    }

    SLAMFile & slamfile  = *slamfilep;

    Sensor::pose_t pose = Eigen::Matrix4f::Identity();

    float depthMapFactor = 1.0;  // default but probably wrong!

    CameraSensor::intrinsics_t intrinsics_rgb;
    DepthSensor::intrinsics_t intrinsics_depth;

    CameraSensor::distortion_coefficients_t distortion_rgb;
    DepthSensor::distortion_coefficients_t distortion_depth;


    if (dirname.find("freiburg1") != std::string::npos) {
        std::cout << "This dataset is assumed to be using freiburg1." << std::endl;
        for (int i = 0; i < 4; i++) {
            intrinsics_rgb[i]   = fr1_intrinsics_rgb[i];
            intrinsics_depth[i] = fr1_intrinsics_depth[i];
            distortion_rgb[i]   = fr1_distortion_rgb[i];
            distortion_depth[i] = fr1_distortion_depth[i];

            depthMapFactor = fr1_DepthMapFactor;
        }
    } else if (dirname.find("freiburg2") != std::string::npos) {
        std::cout << "This dataset is assumed to be using freiburg2." << std::endl;
        for (int i = 0; i < 4; i++) {
            intrinsics_rgb[i]   = fr2_intrinsics_rgb[i];
            intrinsics_depth[i] = fr2_intrinsics_depth[i];
            distortion_rgb[i]   = fr2_distortion_rgb[i];
            distortion_depth[i] = fr2_distortion_depth[i];

            depthMapFactor = fr2_DepthMapFactor;
        }
    } else  {
        std::cout << "Camera calibration might be wrong!" << std::endl;
    }

    DepthSensor::disparity_params_t disparity_params =  {0.001, 0.0};
    DepthSensor::disparity_type_t disparity_type = DepthSensor::affine_disparity;


    /**
     * load Depth
     */
    if (depth && !loadTUMROSDepthData(dirname, bagname, slamfile,
            depthMapFactor, pose, intrinsics_depth, distortion_depth,
            disparity_params, disparity_type)) {
        std::cerr << "error while loading depth information." << std::endl;
        delete slamfilep;
        return nullptr;
    }


    /**
     * load RGB and/or Grey
     * NOTE: TUM rosbag files do not contain grey images - use rgb ones!
     */
    if ((rgb || grey) && !loadTUMROSRGBGreyData(rgb, grey, dirname, bagname,
            slamfile, pose, intrinsics_rgb, distortion_rgb)) {
        std::cerr << "error while loading RGB/Grey information." << std::endl;
        delete slamfilep;
        return nullptr;
    }


    /**
     * load GT
     */
    if (gt && !loadTUMROSGroundTruthData(bagname, slamfile)) {
        std::cerr << "error while loading gt information." << std::endl;
        delete slamfilep;
        return nullptr;
    }


    /**
     * load Accelerometer: This one failed
     */
    if (accelerometer && !loadTUMROSAccelerometerData(bagname, slamfile)) {
        std::cerr << "error while loading Accelerometer information." << std::endl;
        delete slamfilep;
        return nullptr;
    }

    return slamfilep;
}

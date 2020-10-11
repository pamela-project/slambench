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

#include "TUM.h"


using namespace slambench::io ;


bool loadTUMROSBAG_DepthData(const std::string & dirname,
        const std::string & bagname,
        const std::string & topic,
        SLAMFile & file, const Sensor::pose_t & pose,
        const TUMROSBAGReader::image_params_t & image_params,
        const DepthSensor::intrinsics_t & intrinsics,
        const CameraSensor::distortion_coefficients_t & distortion,
        const CameraSensor::distortion_type_t & distortion_type,
        const DepthSensor::disparity_params_t & disparity_params,
        const DepthSensor::disparity_type_t & disparity_type) {

    // allocate new sensor
    auto *depth_sensor = new DepthSensor("Depth");
    if (depth_sensor == nullptr) {
        std::cerr << "error allocating depth sensor" << std::endl;
        return false;
    }
    std::cout << "depth sensor created" << std::endl;

    // populate sensor data
    depth_sensor->Index = file.Sensors.size();
    depth_sensor->Width = image_params.width;
    depth_sensor->Height = image_params.height;
    depth_sensor->FrameFormat = frameformat::Raster;
    depth_sensor->PixelFormat = pixelformat::D_I_16;
    depth_sensor->DisparityType = disparity_type;
    depth_sensor->Description = "Depth";
    depth_sensor->CopyPose(pose);
    depth_sensor->CopyIntrinsics(intrinsics);
    depth_sensor->CopyDisparityParams(disparity_params);
    depth_sensor->DistortionType = distortion_type;
    depth_sensor->CopyRadialTangentialDistortion(distortion);
    depth_sensor->Rate = image_params.rate;
    file.Sensors.AddSensor(depth_sensor);

    // create directory for depth images
    std::string depthdir = dirname + "/depth/";
    if (!boost::filesystem::exists(depthdir)) {
        if (!boost::filesystem::create_directory(depthdir)) {
            std::cerr << "error creating depth directory: "
                    << depthdir << std::endl;
            delete depth_sensor;
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
        delete depth_sensor;
        return false;
    }

    // create query to fetch depth topic messages
    rosbag::View view(bag, rosbag::TopicQuery(topic));

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
                auto dist16 = (ushort) (image_params.depthMapFactor * dist);
                image.at<short>(r, c) = dist16;
            }
        }

        std::stringstream frame_name;
        frame_name << depthdir;
        frame_name << imgi->header.stamp.sec << "."
                << imgi->header.stamp.nsec << ".png";

        if (!cv::imwrite(frame_name.str(), image)) {
            std::cerr << "error writing depth image: " <<
                    frame_name.str() << std::endl;
            delete depth_sensor;
            return false;
        }

        // allocate new depth frame
        auto *depth_frame = new ImageFileFrame();
        if (depth_frame == nullptr) {
            std::cerr << "error creating depth frame" << std::endl;
            delete depth_sensor;
            return false;
        }

        // populate frame and update slambench file
        depth_frame->FrameSensor  = depth_sensor;
        depth_frame->Timestamp.S  = imgi->header.stamp.sec;
        depth_frame->Timestamp.Ns = imgi->header.stamp.nsec;
        depth_frame->filename     = frame_name.str();
        file.AddFrame(depth_frame);
    }

    bag.close();

    return true;
}


bool loadTUMROSBAG_RGBGreyData(
        bool doRGB, bool doGrey, const std::string & dirname,
        const std::string & bagname, const std::string & topic,
        SLAMFile & file, const Sensor::pose_t & pose,
        const TUMROSBAGReader::image_params_t & image_params,
        const CameraSensor::intrinsics_t & intrinsics,
        const CameraSensor::distortion_coefficients_t & distortion,
        const CameraSensor::distortion_type_t & distortion_type) {

    auto *rgb_sensor = new CameraSensor("RGB", CameraSensor::kCameraType);
    if (rgb_sensor == nullptr) {
        std::cerr << "error allocating rgb sensor" << std::endl;
        return false;
    }
    std::cout << "rgb sensor created" << std::endl;

    auto *grey_sensor = new CameraSensor("Grey", CameraSensor::kCameraType);
    if (grey_sensor == nullptr) {
        std::cerr << "error allocating grey sensor" << std::endl;
        delete rgb_sensor;
        return false;
    }
    std::cout << "grey sensor created" << std::endl;

    // populate rgb sensor data
    if (doRGB) {
        rgb_sensor->Index = file.Sensors.size();
        rgb_sensor->Width = image_params.width;
        rgb_sensor->Height = image_params.height;
        rgb_sensor->FrameFormat = frameformat::Raster;
        rgb_sensor->PixelFormat = pixelformat::RGB_III_888;
        rgb_sensor->Description = "RGB";
        rgb_sensor->CopyPose(pose);
        rgb_sensor->CopyIntrinsics(intrinsics);
        rgb_sensor->DistortionType = distortion_type;
        rgb_sensor->CopyRadialTangentialDistortion(distortion);
        rgb_sensor->Rate = image_params.rate;
        file.Sensors.AddSensor(rgb_sensor);
    }

    // populate grey sensor data
    if (doGrey) {
        grey_sensor->Index = file.Sensors.size();
        grey_sensor->Width = image_params.width;
        grey_sensor->Height = image_params.height;
        grey_sensor->FrameFormat = frameformat::Raster;
        grey_sensor->PixelFormat = pixelformat::G_I_8;
        grey_sensor->Description = "Grey";
        grey_sensor->CopyPose(pose);
        grey_sensor->CopyIntrinsics(intrinsics);
        grey_sensor->CopyRadialTangentialDistortion(distortion);
        grey_sensor->DistortionType = distortion_type;
        grey_sensor->Rate = image_params.rate;
        file.Sensors.AddSensor(grey_sensor);
    }

    // create directory for rgb images
    std::string rgbdir = dirname + "/rgb/";
    if (!boost::filesystem::exists(rgbdir)) {
        if (!boost::filesystem::create_directory(rgbdir)) {
            std::cerr << "error creating rgb directory: "
                    << rgbdir << std::endl;
            delete rgb_sensor;
            delete grey_sensor;
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
        delete rgb_sensor;
        delete grey_sensor;
        return false;
    }

    // create query to fetch rgb images topic messages
    // NOTE: TUM rosbag files do not contain grey images - use rgb ones!
    rosbag::View view(bag, rosbag::TopicQuery(topic));

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

        std::stringstream frame_name;
        frame_name << rgbdir;
        frame_name << imgi->header.stamp.sec << "."
                << imgi->header.stamp.nsec << ".png";

        if (!cv::imwrite(frame_name.str(), image)) {
            std::cerr << "error writing rgb image: "
                    << frame_name.str() << std::endl;
            delete rgb_sensor;
            delete grey_sensor;
            return false;
        }

        if (doRGB) {
            // allocate new rgb frame
            auto *rgb_frame = new ImageFileFrame();
            if (rgb_frame == nullptr) {
                std::cerr << "error creating rgb frame" << std::endl;
                delete rgb_sensor;
                delete grey_sensor;
                return false;
            }

            // populate frame and update slambench file
            rgb_frame->FrameSensor = rgb_sensor;
            rgb_frame->Timestamp.S = imgi->header.stamp.sec;
            rgb_frame->Timestamp.Ns = imgi->header.stamp.nsec;
            rgb_frame->filename = frame_name.str();
            file.AddFrame(rgb_frame);
        }

        if (doGrey) {
            // allocate new grey frame
            auto *grey_frame = new ImageFileFrame();
            if (grey_frame == nullptr) {
                std::cerr << "error creating grey frame" << std::endl;
                delete rgb_sensor;
                delete grey_sensor;
                return false;
            }

            // populate frame and update slambench file
            grey_frame->FrameSensor = grey_sensor;
            grey_frame->Timestamp.S = imgi->header.stamp.sec;
            grey_frame->Timestamp.Ns = imgi->header.stamp.nsec;
            grey_frame->filename = frame_name.str();
            file.AddFrame(grey_frame);
        }
    }

    bag.close();

    return true;
}


bool loadTUMROSBAG_GroundTruthData(const std::string & bagname,
        const std::string & topic,
        const TUMROSBAGReader::gt_frame_ids_t & gt_ids,
        SLAMFile & file) {

    auto *gt_sensor = new GroundTruthSensor("GroundTruth");
    if (gt_sensor == nullptr) {
        std::cerr << "error allocating ground truth sensor" << std::endl;
        return false;
    }
    std::cout << "gt sensor created" << std::endl;

    // populate sensor data
    gt_sensor->Index = file.Sensors.size();
    gt_sensor->Description = "GroundTruthSensor";
    file.Sensors.AddSensor(gt_sensor);

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

    // NOTE: initialised to avoid warning
    uint32_t ts_sec = 0;
    uint32_t ts_nsec = 0;

    // open rosbag
    // the ground truth topic contains several transformations
    // ground truth is built from the following composition:
    // optical frame -> rgb frame -> camera -> kinect -> world
    rosbag::Bag bag;
    try {
        bag.open(bagname,rosbag::bagmode::Read);
    }
    catch (...) {
        std::cerr << "error opening gt rosbag: " << bagname << std::endl;
        delete gt_sensor;
        return false;
    }

    // create query to fetch ground truth topic messages
    rosbag::View view(bag, rosbag::TopicQuery(topic));

    for (const auto& msg : view) {
        tf::tfMessage::ConstPtr msgi = msg.instantiate<tf::tfMessage>();
        if (msgi == nullptr) {
            continue;
        }

        for (const auto& msgii : msgi->transforms) {
            if (!r_o_rdy && msgii.child_frame_id == gt_ids.optical) {
                // record the /openni_rgb_frame to /openni_rgb_optical_frame
                // transformation_ only once
                if ((msgii.header.frame_id == gt_ids.rgb)) {
                    r_o_rdy = true;
                    all_rdy = r_o_rdy && c_r_rdy && k_c_rdy;
                    tf::transformMsgToTF(msgii.transform, r_o_trans);
                }
            } else if (!c_r_rdy && msgii.child_frame_id == gt_ids.rgb) {
                // record the /openni_camera to /openni_rgb_frame
                // transformation_ only once
                if ((msgii.header.frame_id == gt_ids.camera)) {
                    c_r_rdy = true;
                    all_rdy = r_o_rdy && c_r_rdy && k_c_rdy;
                    tf::transformMsgToTF(msgii.transform, c_r_trans);
                }
            } else if (!k_c_rdy && msgii.child_frame_id == gt_ids.camera) {
                // record the /kinect to /openni_camera
                // transformation_ only once
                if ((msgii.header.frame_id == gt_ids.kinect)) {
                    k_c_rdy = true;
                    all_rdy = r_o_rdy && c_r_rdy && k_c_rdy;
                    tf::transformMsgToTF(msgii.transform, k_c_trans);
                }
            } else if (msgii.child_frame_id == gt_ids.kinect) {
                // track continuously the /world to /kinect transformation_
                if (msgii.header.frame_id == gt_ids.world) {
                    w_k_new = true;

                    tf::transformMsgToTF(msgii.transform, w_k_trans);

                    // record time stamp to use later in frame
                    ts_sec  = msgii.header.stamp.sec;
                    ts_nsec = msgii.header.stamp.nsec;
                }
            }

            if (all_rdy && w_k_new) {
                w_k_new = false;
                Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
                tf::Transform w_o_trans = w_k_trans * k_c_trans *
                        c_r_trans * r_o_trans;

                tf::Vector3 tr = w_o_trans.getOrigin();
                tf::Matrix3x3 rt = w_o_trans.getBasis();

                // NOTE: float64 (double) to float casts
                pose.block(0, 0, 3, 3)
                        << (float) rt[0][0], (float) rt[0][1], (float) rt[0][2],
                           (float) rt[1][0], (float) rt[1][1], (float) rt[1][2],
                           (float) rt[2][0], (float) rt[2][1], (float) rt[2][2];
                pose.block(0, 3, 3, 1)
                        << (float) tr.x(), (float) tr.y(), (float) tr.z();

                // allocate new gt frame
                auto *gt_frame = new SLAMInMemoryFrame();
                if (gt_frame == nullptr) {
                    std::cerr << "error creating gt frame" << std::endl;
                    delete gt_sensor;
                    return false;
                }

                gt_frame->FrameSensor = gt_sensor;

                // allocate memory for gt data
                gt_frame->Data = malloc(gt_frame->GetSize());
                if (gt_frame->Data == nullptr) {
                    std::cerr << "error allocating memory for gt frame"
                            << std::endl;
                    delete gt_sensor;
                    return false;
                }

                // populate frame data and update slambench file
                gt_frame->Timestamp.S = ts_sec;
                gt_frame->Timestamp.Ns = ts_nsec;
                memcpy(gt_frame->Data, pose.data(), gt_frame->GetSize());
                file.AddFrame(gt_frame);

                // move on to next gt message (skip rest of transformations)
                break;
            }
        }
    }

    bag.close();

    return true;
}


bool loadTUMROSBAG_AccelerometerData(const std::string & bagname,
        const std::string & topic, SLAMFile & file) {

    auto *acc_sensor = new AccelerometerSensor("Accelerometer");
    if (acc_sensor == nullptr) {
        std::cerr << "error allocating accelerometer sensor" << std::endl;
        return false;
    }
    std::cout << "accelerometer sensor created" << std::endl;

    // populate sensor data
    acc_sensor->Index = file.Sensors.size();
    acc_sensor->Description = "AccelerometerSensor";
    file.Sensors.AddSensor(acc_sensor);

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
    rosbag::View view(bag, rosbag::TopicQuery(topic));

    for (const auto& msg : view) {
        sensor_msgs::Imu::ConstPtr msgi = msg.instantiate<sensor_msgs::Imu>();
        if (msgi == nullptr) {
            continue;
        }

        // allocate new accelerometer frame
        auto *acc_frame = new SLAMInMemoryFrame();
        if (acc_frame == nullptr) {
            std::cerr << "error creating acc frame" << std::endl;
            delete acc_sensor;
            return false;
        }

        acc_frame->FrameSensor = acc_sensor;

        // allocate memory for accelerometer data
        acc_frame->Data = malloc(acc_frame->GetSize());
        if (acc_frame->Data == nullptr) {
            std::cerr << "error allocating memory for acc frame" << std::endl;
            delete acc_sensor;
            return false;
        }

        // populate frame and update slambench file
        acc_frame->Timestamp.S = msgi->header.stamp.sec;
        acc_frame->Timestamp.Ns = msgi->header.stamp.nsec;

        // NOTE: float64 (double) to float casts
        ((float *) acc_frame->Data)[0] = (float) (msgi->linear_acceleration).x;
        ((float *) acc_frame->Data)[1] = (float) (msgi->linear_acceleration).y;
        ((float *) acc_frame->Data)[2] = (float) (msgi->linear_acceleration).z;

        file.AddFrame(acc_frame);
    }

    bag.close();

    return true;
}


SLAMFile* TUMROSBAGReader::GenerateSLAMFile () {

    // NOTE: can accelerometer be used on its own?
    if (!(grey || rgb || depth || accelerometer)) {
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

    image_params_t image_params = get_image_params();

    /**
     * get sensor parameters
     *
     * some of these parameters depend on the actual kinect device used
     *
     */
    DepthSensor::disparity_params_t         disparity_params;
    DepthSensor::disparity_type_t           disparity_type;
    CameraSensor::intrinsics_t              intrinsics_rgb;
    DepthSensor::intrinsics_t               intrinsics_depth;
    CameraSensor::distortion_coefficients_t distortion_rgb;
    DepthSensor::distortion_coefficients_t  distortion_depth;
    CameraSensor::distortion_type_t         distortion_type;

    uint32_t kinect = get_sensor_params(disparity_params, disparity_type,
                              intrinsics_rgb, intrinsics_depth,
                              distortion_rgb, distortion_depth,
                              distortion_type);

    if (kinect) {
        std::cout << "using freiburg" << kinect	<< " camera calibration parameters" << std::endl;
    } else {
        std::cout << "using default camera calibration parameters" << std::endl;
        std::cout << "warning: camera calibration might be wrong!" << std::endl;
    }


    /**
     * load Depth
     */
    if (depth && !loadTUMROSBAG_DepthData(dirname, bagname, depth_topic,
            slamfile, pose, image_params, intrinsics_depth, distortion_depth,
            distortion_type, disparity_params, disparity_type)) {
        std::cerr << "error loading depth data." << std::endl;
        delete slamfilep;
        return nullptr;
    }


    /**
     * load RGB and/or Grey
     * NOTE: TUM rosbag files do not contain grey images - use rgb ones!
     */
    if ((rgb || grey) && !loadTUMROSBAG_RGBGreyData(rgb, grey,
            dirname, bagname, rgb_topic, slamfile, pose, image_params,
            intrinsics_rgb, distortion_rgb, distortion_type)) {
        std::cerr << "error loading RGB/Grey data." << std::endl;
        delete slamfilep;
        return nullptr;
    }


    /**
     * load GT
     */
    if (gt && !loadTUMROSBAG_GroundTruthData(bagname, gt_topic,
            gt_frame_ids, slamfile)) {
        std::cerr << "error loading gt data." << std::endl;
        delete slamfilep;
        return nullptr;
    }


    /**
     * load Accelerometer
     */
    if (accelerometer && !loadTUMROSBAG_AccelerometerData(bagname,
            acc_topic, slamfile)) {
        std::cerr << "error loading Accelerometer data."
                << std::endl;
        delete slamfilep;
        return nullptr;
    }

    return slamfilep;
}

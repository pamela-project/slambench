/*

 Copyright (c) 2017-2020 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 The interface with ROS datasets (rosbags) is supported by the RAIN Hub, which is funded by
 the Industrial Strategy Challenge Fund, part of the government’s modern Industrial Strategy.
 The fund is delivered by UK Research and Innovation and managed by EPSRC [EP/R026084/1].

 This code is licensed under the MIT License.

 */


#include "../dataset-tools/include/TUM-ROS.h"

#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/sensor/AccelerometerSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/format/PointCloud.h>
#include <Eigen/Eigen>


#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <boost/foreach.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>

#define foreach BOOST_FOREACH

using namespace slambench::io ;



bool loadTUMROSDepthData(const std::string &dirname, const std::string &bagname, SLAMFile &file, const Sensor::pose_t &pose, const DepthSensor::intrinsics_t &intrinsics,const CameraSensor::distortion_coefficients_t &distortion,  const DepthSensor::disparity_params_t &disparity_params, const DepthSensor::disparity_type_t &disparity_type) {

    // populate sensor data
	auto *depth_sensor = new DepthSensor("Depth");
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

	// open rosbag file
    rosbag::Bag bag;
    try {
        bag.open(bagname,rosbag::bagmode::Read);
    }
    catch (...) {
        std::cout << "Error opening depth rosbag: " << bagname << std::endl;
        return false;
    }

    // create directory for depth images
    std::string depthdir = dirname + "/depth/";
    if (!boost::filesystem::exists(depthdir)) {
        if (!boost::filesystem::create_directory(depthdir)) {
            std::cout << "Error creating depth directory: " << depthdir << std::endl;
            return false;
        }
    }

    // create query to fetch depth topic messages
    std::vector<std::string> topics;
    topics.push_back(std::string("/camera/depth/image"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // initialised to avoid warnings!
    uint32_t sec  = 0;
    uint32_t nsec = 0;

    // produce png image for every depth message
    foreach(rosbag::MessageInstance const msg, view) {
        sensor_msgs::Image::ConstPtr imgi = msg.instantiate<sensor_msgs::Image>();
        cv::Mat imgo = cv::Mat(imgi->height, imgi->width, CV_32FC1,
                                const_cast<uchar*>(&imgi->data[0]), imgi->step);
        cv::Mat image = cv::Mat(imgi->height, imgi->width, CV_16UC1);
        for (uint r = 0; r < imgi->height; r++) {
            for (uint c = 0; c < imgi->width; c++) {
                float dist = imgo.at<float>(r * imgi->width + c);
                auto dist16 = (unsigned short) (5000 * dist);
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
            std::cout << "Error writing depth image: " << frame_name.str() << std::endl;
            return false;
        }

        // update slambench file with new frame
        auto *depth_frame = new ImageFileFrame();
        depth_frame->FrameSensor  = depth_sensor;
        depth_frame->Timestamp.S  = sec;
        depth_frame->Timestamp.Ns = nsec;
        depth_frame->Filename     = frame_name.str();
        file.AddFrame(depth_frame);
    }

    bag.close();

    return true;
}


bool loadTUMROSRGBData(const std::string &dirname, const std::string &bagname, SLAMFile &file, const Sensor::pose_t &pose, const CameraSensor::intrinsics_t &intrinsics, const CameraSensor::distortion_coefficients_t &distortion) {

    // populate sensor data
	auto *rgb_sensor = new CameraSensor("RGB", CameraSensor::kCameraType);
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
	rgb_sensor->Index =file.Sensors.size();
	rgb_sensor->Rate = 30.0;
	file.Sensors.AddSensor(rgb_sensor);

    // open rosbag file
    rosbag::Bag bag;
    try {
        bag.open(bagname, rosbag::bagmode::Read);
    }
    catch (...) {
        std::cout << "Error opening rgb rosbag: " << bagname << std::endl;
        return false;
    }

    // create directory for rgb images
    std::string rgbdir = dirname + "/rgb/";
    if (!boost::filesystem::exists(rgbdir)) {
        if (!boost::filesystem::create_directory(rgbdir)) {
            std::cout << "Error creating rgb directory: " << rgbdir << std::endl;
            return false;
        }
    }

    // create query to fetch rgb images topic messages
    std::vector<std::string> topics;
    topics.push_back(std::string("/camera/rgb/image_color"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // initialised to avoid warnings!
    uint32_t sec  = 0;
    uint32_t nsec = 0;

    // produce png image for every rgb message
    foreach(rosbag::MessageInstance const msg, view) {
        sensor_msgs::Image::ConstPtr imgi = msg.instantiate<sensor_msgs::Image>();
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
        // TUM RGB-D datasets use 6-digit nanosec timestamps
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
            std::cout << "Error writing rgb image: " << frame_name.str() << std::endl;
            return false;
        }

        // update slambench file with new frame
        auto *rgb_frame = new ImageFileFrame();
        rgb_frame->FrameSensor = rgb_sensor;
        rgb_frame->Timestamp.S = sec;
        rgb_frame->Timestamp.Ns = nsec;
        rgb_frame->Filename = frame_name.str();
        file.AddFrame(rgb_frame);
    }

    bag.close();

    return true;
}


bool loadTUMROSGreyData(const std::string &dirname, const std::string &bagname, SLAMFile &file, const Sensor::pose_t &pose, const CameraSensor::intrinsics_t &intrinsics, const CameraSensor::distortion_coefficients_t &distortion) {

	auto *grey_sensor = new CameraSensor("Grey", CameraSensor::kCameraType);
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
	grey_sensor->Index =file.Sensors.size();
	grey_sensor->Rate = 30.0;
    file.Sensors.AddSensor(grey_sensor);

    // open rosbag file
    rosbag::Bag bag;
    try {
        bag.open(bagname,rosbag::bagmode::Read);
    }
    catch (...) {
        std::cout << "Error opening rgb/(grey)rosbag: " << bagname << std::endl;
        return false;
    }

    // create query to fetch rgb images topic messages
    // NOTE: TUM rosbag files do not contain grey images - use rgb ones!
    std::vector<std::string> topics;
    topics.push_back(std::string("/camera/rgb/image_color"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // initialised to avoid warnings!
    uint32_t sec  = 0;
    uint32_t nsec = 0;

    // track png image for every message
    foreach(rosbag::MessageInstance const msg, view) {
        sensor_msgs::Image::ConstPtr imgi = msg.instantiate<sensor_msgs::Image>();
        std::stringstream frame_name;

        // record time stamp with adjusted precision
        // TUM RGB-D datasets use 6-digit nanosec timestamps
        sec = imgi->header.stamp.sec;
        nsec = (imgi->header.stamp.nsec + 500)/1000;
        if (nsec >= 1000000) {
            sec++;
            nsec = 0;
        }

        frame_name << dirname << "/rgb/";
        frame_name << sec << ".";
        frame_name << std::setw(6) << std::setfill('0') << nsec << ".png";

        // update slambench file with new frame
        auto *grey_frame = new ImageFileFrame();
        grey_frame->FrameSensor = grey_sensor;
        grey_frame->Timestamp.S = sec;
        grey_frame->Timestamp.Ns = nsec;
        grey_frame->Filename = frame_name.str();
        file.AddFrame(grey_frame);
    }

    bag.close();

    return true;
}


bool loadTUMROSGroundTruthData(const std::string &bagname, SLAMFile &file) {

	auto *gt_sensor = new GroundTruthSensor("GroundTruth");
	gt_sensor->Index = file.Sensors.size();
	gt_sensor->Description = "GroundTruthSensor";
	file.Sensors.AddSensor(gt_sensor);

	if(!gt_sensor) {
		std::cout << "gt sensor not found..." << std::endl;
		return false;
	} else {
		std::cout << "gt sensor created..." << std::endl;
	}

    // open rosbag
    rosbag::Bag bag;
	try {
        bag.open(bagname,rosbag::bagmode::Read);
    }
	catch (...) {
        std::cout << "Error opening gt rosbag: " << bagname << std::endl;
        return false;
	}

    // create query to fetch ground truth topic messages
    std::vector<std::string> topics;
    topics.push_back(std::string("/tf"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

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

    foreach(rosbag::MessageInstance const msg, view) {
        tf::tfMessage::ConstPtr msgi = msg.instantiate<tf::tfMessage>();

        if (msgi != nullptr) {
            for (unsigned long i = 0; i < msgi->transforms.size(); i++) {
                geometry_msgs::TransformStamped msgii = msgi->transforms[i];
                if (!r_o_rdy && msgii.child_frame_id == opt_str) {
                    // record once the /openni_rgb_frame to /openni_rgb_optical_frame transformation
                    if ((msgii.header.frame_id == rgb_str)) {
                        r_o_rdy = true;
                        all_rdy = r_o_rdy && c_r_rdy && k_c_rdy;
                        tf::transformMsgToTF(msgii.transform, r_o_trans);
                    }
                } else if (!c_r_rdy && msgii.child_frame_id == rgb_str) {
                    // record once the /openni_camera to /openni_rgb_frame transformation
                    if ((msgii.header.frame_id == camera_str)) {
                        c_r_rdy = true;
                        all_rdy = r_o_rdy && c_r_rdy && k_c_rdy;
                        tf::transformMsgToTF(msgii.transform, c_r_trans);
                    }
                } else if (!k_c_rdy && msgii.child_frame_id == camera_str) {
                    // record once the /kinect to /openni_camera transformation
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
                        // TUM RGB-D datasets use 4-digit ground truth nanosec timestamps
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
                    tf::Transform w_o_trans = w_k_trans * k_c_trans * c_r_trans * r_o_trans;

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
                    gt_frame->FrameSensor = gt_sensor;
                    gt_frame->Timestamp.S = sec;
                    gt_frame->Timestamp.Ns = nsec;
                    gt_frame->Data = malloc(gt_frame->GetSize());

                    memcpy(gt_frame->Data, pose.data(), gt_frame->GetSize());

                    file.AddFrame(gt_frame);

                    break;
                }
            }
        }
    }

    bag.close();

	return true;
}


bool loadTUMROSAccelerometerData(const std::string &dirname, SLAMFile &file) {

	auto *accelerometer_sensor = new AccelerometerSensor("Accelerometer");
	accelerometer_sensor->Index = file.Sensors.size();
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

		if (line.empty()) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^\\s*#.*$"))) {
			continue;
		} else if (boost::regex_match(line,match,boost::regex("^([0-9]+)[.]([0-9]+)\\s+([-0-9.]+)\\s+([-0-9.]+)\\s+([-0-9.]+)$"))) {

		  int timestampS = std::stoi(match[1]);
		  int timestampNS = std::stoi(match[2]) *  std::pow ( 10, 9 - match[2].length());
		  float ax =  std::stof(match[3]);
		  float ay =  std::stof(match[4]);
		  float az =  std::stof(match[5]);

		  auto *accelerometer_frame = new SLAMInMemoryFrame();
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


SLAMFile* TUMROSReader::GenerateSLAMFile () {

	if(!(grey || rgb || depth)) {
		std::cerr <<  "No sensors defined" << std::endl;
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
	SLAMFile & slamfile  = *slamfilep;

	Sensor::pose_t pose = Eigen::Matrix4f::Identity();

	//////  Default are freiburg1

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
		}

	} else if (dirname.find("freiburg2") != std::string::npos) {
		std::cout << "This dataset is assumed to be using freiburg2." << std::endl;
		for (int i = 0; i < 4; i++) {
			intrinsics_rgb[i]   = fr2_intrinsics_rgb[i];
			intrinsics_depth[i] = fr2_intrinsics_depth[i];
			distortion_rgb[i]   = fr2_distortion_rgb[i];
			distortion_depth[i] = fr2_distortion_depth[i];
		}

	} else  {
		std::cout << "Camera calibration might be wrong !." << std::endl;
	}


	DepthSensor::disparity_params_t disparity_params =  {0.001, 0.0};
	DepthSensor::disparity_type_t disparity_type = DepthSensor::affine_disparity;


	/**
	 * load Depth
	 */
	if(depth && !loadTUMROSDepthData(dirname, bagname, slamfile, pose, intrinsics_depth, distortion_depth, disparity_params, disparity_type)) {
		std::cout << "Error while loading depth information." << std::endl;
		delete slamfilep;
		return nullptr;
	}


	/**
	 * load Grey
	 */
	if(grey && !loadTUMROSGreyData(dirname, bagname, slamfile, pose, intrinsics_rgb, distortion_rgb)) {
		std::cout << "Error while loading Grey information." << std::endl;
		delete slamfilep;
		return nullptr;
	}


	/**
	 * load RGB
	 */
	if(rgb && !loadTUMROSRGBData(dirname, bagname, slamfile, pose, intrinsics_rgb, distortion_rgb)) {
		std::cout << "Error while loading RGB information." << std::endl;
		delete slamfilep;
		return nullptr;
	}


	/**
	 * load GT
	 */
	if(gt && !loadTUMROSGroundTruthData(bagname, slamfile)) {
		std::cout << "Error while loading gt information." << std::endl;
		delete slamfilep;
		return nullptr;
	}


	/**
	 * load Accelerometer: This one failed
	 */
	if(accelerometer && !loadTUMROSAccelerometerData(dirname, slamfile)) {
		std::cout << "Error while loading Accelerometer information." << std::endl;
		delete slamfilep;
		return nullptr;
	}

	return slamfilep;
	}
/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "../dataset-tools/include/TUM-ROS.h"

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

#include <boost/foreach.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>

#define foreach BOOST_FOREACH

using namespace slambench::io ;



/*
 *
 * The dataset folder contains :
 * > accelerometer.txt  depth  depth.txt  groundtruth.txt  rgb  rgb.txt
 *
 */

bool analyseTUMFolder(const std::string &dirname) {

	static const std::vector<std::string> requirements = {
			"accelerometer.txt",
			"rgb.txt",
			"rgb",
			"depth.txt",
			"depth",
			"groundtruth.txt"
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


bool loadTUMDepthData(const std::string &dirname , SLAMFile &file, const Sensor::pose_t &pose, const DepthSensor::intrinsics_t &intrinsics,const CameraSensor::distortion_coefficients_t &distortion,  const DepthSensor::disparity_params_t &disparity_params, const DepthSensor::disparity_type_t &disparity_type) {

    // populate sensor data
	DepthSensor *depth_sensor = new DepthSensor("Depth");
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
        bag.open(dirname + "/" + "rgbd_dataset_freiburg2_pioneer_slam.bag",
                 rosbag::bagmode::Read);
    }
    catch (...) {
        std::cout << "Error opening rosbag" << std::endl;
        return false;
    }

    // create query to fetch depth topic messages
    std::vector<std::string> topics;
    topics.push_back(std::string("/camera/depth/image"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // produce png image for every depth message
    foreach(rosbag::MessageInstance const msg, view) {
        sensor_msgs::Image::ConstPtr imgi = msg.instantiate<sensor_msgs::Image>();
        cv::Mat imgo = cv::Mat(imgi->height, imgi->width, CV_32FC1,
                                const_cast<uchar*>(&imgi->data[0]), imgi->step);
        cv::Mat image = cv::Mat(imgi->height, imgi->width, CV_16UC1);
        for (uint r = 0; r < imgi->height; r++) {
            for (uint c = 0; c < imgi->width; c++) {
                float dist = imgo.at<float>(r * imgi->width + c);
                unsigned short dist16 = (unsigned short) (5000 * dist);
                image.at<short>(r, c) = dist16;
            }
        }
        std::stringstream frame_name;
        frame_name << dirname << "/depth/";
        frame_name << imgi->header.stamp.sec << ".";
        frame_name << std::setw(6) << std::setfill('0') << imgi->header.stamp.nsec << ".png";
        cv::imwrite(frame_name.str(), image);

        // update slambench file with new frame
        ImageFileFrame *depth_frame = new ImageFileFrame();
        depth_frame->FrameSensor  = depth_sensor;
        depth_frame->Timestamp.S  = imgi->header.stamp.sec;
        depth_frame->Timestamp.Ns = imgi->header.stamp.nsec;
        depth_frame->Filename     = frame_name.str();
        file.AddFrame(depth_frame);
    }
	return true;
}


bool loadTUMRGBData(const std::string &dirname , SLAMFile &file, const Sensor::pose_t &pose, const CameraSensor::intrinsics_t &intrinsics, const CameraSensor::distortion_coefficients_t &distortion) {

    // populate sensor data
	CameraSensor *rgb_sensor = new CameraSensor("RGB", CameraSensor::kCameraType);
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
        bag.open(dirname + "/" + "rgbd_dataset_freiburg2_pioneer_slam.bag",
                 rosbag::bagmode::Read);
    }
    catch (...) {
        std::cout << "Error opening rosbag" << std::endl;
        return false;
    }

    // create query to fetch depth topic messages
    std::vector<std::string> topics;
    topics.push_back(std::string("/camera/rgb/image_color"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

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
        std::stringstream frame_name;
        frame_name << dirname << "/rgb/";
        frame_name << imgi->header.stamp.sec << ".";
        frame_name << std::setw(6) << std::setfill('0') << imgi->header.stamp.nsec << ".png";
        cv::imwrite(frame_name.str(), image);

        // update slambench file with new frame
        ImageFileFrame *rgb_frame = new ImageFileFrame();
        rgb_frame->FrameSensor = rgb_sensor;
        rgb_frame->Timestamp.S = imgi->header.stamp.sec;
        rgb_frame->Timestamp.Ns = imgi->header.stamp.nsec;
        rgb_frame->Filename = frame_name.str();
        file.AddFrame(rgb_frame);
    }

	return true;
}

bool loadTUMGreyData(const std::string &dirname , SLAMFile &file, const Sensor::pose_t &pose, const CameraSensor::intrinsics_t &intrinsics, const CameraSensor::distortion_coefficients_t &distortion) {

	CameraSensor *grey_sensor = new CameraSensor("Grey", CameraSensor::kCameraType);
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
        bag.open(dirname + "/" + "rgbd_dataset_freiburg2_pioneer_slam.bag",
                 rosbag::bagmode::Read);
    }
    catch (...) {
        std::cout << "Error opening rosbag" << std::endl;
        return false;
    }

    // create query to fetch depth topic messages
    std::vector<std::string> topics;
    topics.push_back(std::string("/camera/rgb/image_color"));
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // track png image for every message
    foreach(rosbag::MessageInstance const msg, view) {
        sensor_msgs::Image::ConstPtr imgi = msg.instantiate<sensor_msgs::Image>();
        std::stringstream frame_name;
        frame_name << dirname << "/rgb/";
        frame_name << imgi->header.stamp.sec << ".";
        frame_name << std::setw(6) << std::setfill('0') << imgi->header.stamp.nsec << ".png";

        // update slambench file with new frame
        ImageFileFrame *grey_frame = new ImageFileFrame();
        grey_frame->FrameSensor = grey_sensor;
        grey_frame->Timestamp.S = imgi->header.stamp.sec;
        grey_frame->Timestamp.Ns = imgi->header.stamp.nsec;
        grey_frame->Filename = frame_name.str();
        file.AddFrame(grey_frame);
    }

	return true;
}


bool loadTUMGroundTruthData(const std::string &dirname , SLAMFile &file) {

	GroundTruthSensor *gt_sensor = new GroundTruthSensor("GroundTruth");
	gt_sensor->Index = file.Sensors.size();
	gt_sensor->Description = "GroundTruthSensor";
	file.Sensors.AddSensor(gt_sensor);

	if(!gt_sensor) {
		std::cout << "gt sensor not found..." << std::endl;
		return false;
	} else {
		std::cout << "gt sensor created..." << std::endl;
	}

    rosbag::Bag bag;
	try {
        bag.open(dirname + "/" + "rgbd_dataset_freiburg2_pioneer_slam.bag",
                 rosbag::bagmode::Read);
    }
	catch (...) {
        std::cout << "Error opening rosbag" << std::endl;
        return false;
	}

    std::vector<std::string> topics;
    topics.push_back(std::string("/tf"));

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    ros::Time w_k_stamp;

    std::string world_str ("/world");
    std::string kinect_str ("/kinect");
    std::string camera_str ("/openni_camera");
    std::string rgb_str ("/openni_rgb_frame");
    std::string opt_str ("/openni_rgb_optical_frame");

    tf::Transform w_k_trans;  // constant transformation
    tf::Transform k_c_trans;  // constant transformation
    tf::Transform c_r_trans;  // constant transformation
    tf::Transform r_o_trans;  // constant transformation

    bool w_k_new = false;
    bool k_c_rdy = false;
    bool c_r_rdy = false;
    bool r_o_rdy = false;
    bool all_rdy = false;

    uint32_t sec, nsec;

    foreach(rosbag::MessageInstance const msg, view) {
        tf::tfMessage::ConstPtr msgi = msg.instantiate<tf::tfMessage>();

        if (msgi != NULL) {
            for (unsigned int i = 0; i < msgi->transforms.size(); i++) {
                geometry_msgs::TransformStamped msgii = msgi->transforms[i];
                if (!r_o_rdy && msgii.child_frame_id.compare(opt_str) == 0) {
                    // record once the /openni_rgb_frame to /openni_rgb_optical_frame transformation
                    if ((msgii.header.frame_id.compare(rgb_str) == 0)) {
                        r_o_rdy = true;
                        all_rdy = r_o_rdy && c_r_rdy && k_c_rdy;
                        tf::transformMsgToTF(msgii.transform, r_o_trans);
                    }
                } else if (!c_r_rdy && msgii.child_frame_id.compare(rgb_str) == 0) {
                    // record once the /openni_camera to /openni_rgb_frame transformation
                    if ((msgii.header.frame_id.compare(camera_str) == 0)) {
                        c_r_rdy = true;
                        all_rdy = r_o_rdy && c_r_rdy && k_c_rdy;
                        tf::transformMsgToTF(msgii.transform, c_r_trans);
                    }
                } else if (!k_c_rdy && msgii.child_frame_id.compare(camera_str) == 0) {
                    // record once the /kinect to /openni_camera transformation
                    if ((msgii.header.frame_id.compare(kinect_str) == 0)) {
                        k_c_rdy = true;
                        all_rdy = r_o_rdy && c_r_rdy && k_c_rdy;
                        tf::transformMsgToTF(msgii.transform, k_c_trans);
                    }
                } else if (msgii.child_frame_id.compare(kinect_str) == 0) {
                    // track continuously the /world to /kinect transformation
                    if (msgii.header.frame_id.compare(world_str) == 0) {
                        w_k_new = true;
                        w_k_stamp = msgii.header.stamp;
                        sec = w_k_stamp.sec;
                        nsec = w_k_stamp.nsec;
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

                    SLAMInMemoryFrame *gt_frame = new SLAMInMemoryFrame();
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


bool loadTUMAccelerometerData(const std::string &dirname , SLAMFile &file) {

	AccelerometerSensor *accelerometer_sensor = new AccelerometerSensor("Accelerometer");
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






SLAMFile* TUMROSReader::GenerateSLAMFile () {

	if(!(grey || rgb || depth)) {
		std::cerr <<  "No sensors defined\n";
		return nullptr;
	}

	std::string dirname = input;

	if (!analyseTUMFolder(dirname))	{
		std::cerr << "Invalid folder." << std::endl;
		return nullptr;
	}


	SLAMFile * slamfilep = new SLAMFile();
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



	DepthSensor::disparity_params_t disparity_params =  {0.001,0.0};
	DepthSensor::disparity_type_t disparity_type = DepthSensor::affine_disparity;


	/**
	 * load Depth
	 */

	if(depth && !loadTUMDepthData(dirname, slamfile,pose,intrinsics_depth,distortion_depth,disparity_params,disparity_type)) {
		std::cout << "Error while loading depth information." << std::endl;
		delete slamfilep;
		return nullptr;

	}


	/**
	 * load Grey
	 */

	if(grey && !loadTUMGreyData(dirname, slamfile,pose,intrinsics_rgb,distortion_rgb)) {
		std::cout << "Error while loading Grey information." << std::endl;
		delete slamfilep;
		return nullptr;

	}


	/**
	 * load RGB
	 */

	if(rgb && !loadTUMRGBData(dirname, slamfile,pose,intrinsics_rgb,distortion_rgb)) {
		std::cout << "Error while loading RGB information." << std::endl;
		delete slamfilep;
		return nullptr;

	}


	/**
	 * load GT
	 */
	if(gt && !loadTUMGroundTruthData(dirname, slamfile)) {
		std::cout << "Error while loading gt information." << std::endl;
		delete slamfilep;
		return nullptr;

	}


	/**
	 * load Accelerometer: This one failed
	 */
	if(accelerometer && !loadTUMAccelerometerData(dirname, slamfile)) {
		std::cout << "Error while loading Accelerometer information." << std::endl;
		delete slamfilep;
		return nullptr;

	}


	return slamfilep;
	}







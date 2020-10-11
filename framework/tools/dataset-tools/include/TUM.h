/*

 Copyright (c) 2017-2020 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 The development of the interface with ROS datasets (rosbags) is supported
 by the RAIN Hub, which is funded by the Industrial Strategy Challenge Fund,
 part of the UK government’s modern Industrial Strategy. The fund is
 delivered by UK Research and Innovation and managed by EPSRC [EP/R026084/1].

 This code is licensed under the MIT License.

 */


#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_TUM_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_TUM_H_


#include <ParameterManager.h>
#include <ParameterComponent.h>
#include <Parameters.h>

#include <io/sensor/Sensor.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>

#include "DatasetReader.h"


namespace slambench {
    namespace io {
        class TUMReader : public DatasetReader {
        enum DatasetOrigin { Default = 0, Freiburg1, Freiburg2, Freiburg3, ETHI };
        private:
            /****
             * Taken from:
             * https://vision.in.tum.de/_media/spezial/bib/sturm12iros.pdf
             * data recorded on three Microsoft Xbox Kinect sensors
             * (freiburg1, freiburg2 and freiburg3)
             * at full resolution (640×480) and full frame rate (30 Hz)
             * depth images are expected to be scaled by a factor of 5000 [5000 : 1 m]
             * (no indication as to the reason).
             *
             */
            static constexpr image_params_t fr_image_params
                    = { 640, 480, 30.0, 5000.0 };

            /****
             * source of these parameters unknown
             */
            static constexpr DepthSensor::disparity_params_t fr_disparity_params
                    =  { 0.0002, 0.0 };
            static const DepthSensor::disparity_type_t fr_disparity_type
                    = DepthSensor::affine_disparity;

            /****
             * intrinsic parameters: focal length and optical center {fx, fy, cx, cy}
             * expressed as fractions of image width (fx and cx) and height (fy and cy)
             * Taken from:
             * https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats#intrinsic_camera_calibration_of_the_kinect
             * data recorded on three Microsoft Xbox Kinect sensors
             * (freiburg1, freiburg2 and freiburg3)
                Camera          fx 	    fy 	    cx 	    cy
                (ROS default) 	525.0 	525.0 	319.5 	239.5
                Freiburg 1 RGB 	517.3 	516.5 	318.6 	255.3
                Freiburg 2 RGB 	520.9 	521.0 	325.1 	249.7
                Freiburg 3 RGB 	535.4 	539.2 	320.1 	247.6
                ETHI       RGB 	538.7  	540.7 	319.2 	233.6

                Camera  	    fx  	fy 	    cx 	    cy
                Freiburg 1 IR 	591.1 	590.1 	331.0 	234.0
                Freiburg 2 IR 	580.8 	581.8 	308.8 	253.0
                Freiburg 3 IR 	567.6 	570.2 	324.7 	250.1
                ETHI       IR 	538.7 	540.7 	319.2 	233.6
             *
             */
            static constexpr CameraSensor::intrinsics_t fr1_intrinsics_rgb
                    = { 517.3 / fr_image_params.width, 516.5 / fr_image_params.height,
                        318.6 / fr_image_params.width, 255.3 / fr_image_params.height };

            static constexpr CameraSensor::intrinsics_t fr2_intrinsics_rgb
                    = { 520.9 / fr_image_params.width, 521.0 / fr_image_params.height,
                        325.1 / fr_image_params.width, 249.7 / fr_image_params.height };

            static constexpr CameraSensor::intrinsics_t fr3_intrinsics_rgb
                    = { 535.4 / fr_image_params.width, 539.2 / fr_image_params.height,
                        320.1 / fr_image_params.width, 247.6 / fr_image_params.height };
            static constexpr CameraSensor::intrinsics_t ethi_intrinsics_rgb
                    = {538.7 / fr_image_params.width, 540.7 / fr_image_params.height,
                       319.2 / fr_image_params.width, 233.6 / fr_image_params.height};

            // default ROS values -- use with caution
            static constexpr CameraSensor::intrinsics_t default_intrinsics_rgb
                    = { 525.0 / fr_image_params.width, 525.0 / fr_image_params.height,
                        319.5 / fr_image_params.width, 239.5 / fr_image_params.height };

            // depth images are taken by the IR camera
            static constexpr DepthSensor::intrinsics_t fr1_intrinsics_depth
                    = { 591.1 / fr_image_params.width, 590.1 / fr_image_params.height,
                        331.0 / fr_image_params.width, 234.0 / fr_image_params.height };

            static constexpr DepthSensor::intrinsics_t fr2_intrinsics_depth
                    = { 580.8 / fr_image_params.width, 581.8 / fr_image_params.height,
                        308.8 / fr_image_params.width, 253.0 / fr_image_params.height };

            static constexpr DepthSensor::intrinsics_t fr3_intrinsics_depth
                    = { 567.6 / fr_image_params.width, 570.2 / fr_image_params.height,
                        324.7 / fr_image_params.width, 250.1 / fr_image_params.height };

            static constexpr DepthSensor::intrinsics_t  ethi_intrinsics_depth
                    = { 538.7 / fr_image_params.width, 540.7 / fr_image_params.height,
                        319.2 / fr_image_params.width, 233.6 / fr_image_params.height};

            // default ROS values -- use with caution
            static constexpr DepthSensor::intrinsics_t default_intrinsics_depth
                    = { 525.0 / fr_image_params.width, 525.0 / fr_image_params.height,
                        319.5 / fr_image_params.width, 239.5 / fr_image_params.height };


            /****
             * distortion parameters: radial and tangential factors {k1, k2, p1, p2, k3}
             * Taken from:
             * https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats#intrinsic_camera_calibration_of_the_kinect
             * data recorded on three Microsoft Xbox Kinect sensors (freiburg1, freiburg2 and freiburg3)
                Camera          d0 	    d1 	    d2  	d3      d4
                (ROS default) 	0.0 	0.0 	0.0 	0.0 	0.0
                Freiburg 1 RGB 	0.2624	-0.9531	-0.0054	0.0026 	1.1633
                Freiburg 2 RGB 	0.2312	-0.7849	-0.0033	-0.0001 0.9172
                Freiburg 3 RGB 	0 	    0 	    0 	    0 	    0
                ETHI       RGB 	0 	    0 	    0 	    0 	    0

                Camera  	    d0 	    d1 	    d2 	    d3 	    d4
                Freiburg 1 IR 	-0.0410	0.3286	0.0087	0.0051 	-0.5643
                Freiburg 2 IR 	-0.2297	1.4766	0.0005	-0.0075 -3.4194
                Freiburg 3 IR 	0 	    0 	    0 	    0 	    0
                ETHI       IR 	0 	    0 	    0 	    0 	    0
             *
             */
            static const CameraSensor::distortion_type_t camera_distortion_type
                    = CameraSensor::RadialTangential;

            // these numbers taken from ORBSLAM2 examples
            // (appear to have higher precision than table above)
            static constexpr CameraSensor::distortion_coefficients_t fr1_distortion_rgb
                    = { 0.262383, -0.953104, -0.005358, 0.002628 , 1.163314 };

            static constexpr CameraSensor::distortion_coefficients_t fr2_distortion_rgb
                    = { 0.231222, -0.784899, -0.003257, -0.000105, 0.917205 };

            static constexpr CameraSensor::distortion_coefficients_t fr3_distortion_rgb
                    = { 0.0, 0.0, 0.0, 0.0, 0.0 };

            static constexpr CameraSensor::distortion_coefficients_t ethi_distortion_rgb
                    = { 0.0, 0.0, 0.0, 0.0, 0.0 };

            // default ROS values -- use with caution
            static constexpr CameraSensor::distortion_coefficients_t default_distortion_rgb
                    = { 0.0, 0.0, 0.0, 0.0, 0.0 };

            // depth images are pre-registered to the RGB images, thus rectifying
            // depth images based on intrinsic parameters is not straight forward
            // use RGB (not IR) distortion parameters as an approximation
            static constexpr DepthSensor::distortion_coefficients_t fr1_distortion_depth
                    = { 0.262383 , -0.953104, -0.005358, 0.002628 , 1.163314 };

            static constexpr DepthSensor::distortion_coefficients_t fr2_distortion_depth
                    = { 0.231222, -0.784899, -0.003257, -0.000105, 0.917205 };

            static constexpr DepthSensor::distortion_coefficients_t fr3_distortion_depth
                    = { 0.0, 0.0, 0.0, 0.0, 0.0 };

            static constexpr DepthSensor::distortion_coefficients_t  ethi_distortion_depth
                    = { 0.0, 0.0, 0.0, 0.0, 0.0 };

            // default ROS values -- use with caution
            static constexpr DepthSensor::distortion_coefficients_t default_distortion_depth
                    = { 0.0, 0.0, 0.0, 0.0, 0.0 };

        public :
            std::string input;
            bool grey = true, rgb = true, depth = true, gt = true, accelerometer = true;

            explicit TUMReader(std::string name) : DatasetReader(std::move(name)) {
                this->addParameter(TypedParameter<std::string>("i",
                        "input-directory", "path of the TUM dataset directory",
                        &this->input, nullptr));
                this->addParameter(TypedParameter<bool>("grey", "grey",
                        "set to true or false to specify if the GREY stream need to be include in the slam file.",
                        &this->grey, nullptr));
                this->addParameter(TypedParameter<bool>("rgb", "rgb",
                        "set to true or false to specify if the RGB stream need to be include in the slam file.",
                        &this->rgb, nullptr));
                this->addParameter(TypedParameter<bool>("depth", "depth",
                        "set to true or false to specify if the DEPTH stream need to be include in the slam file.",
                        &this->depth, nullptr));
                this->addParameter(TypedParameter<bool>("gt", "gt",
                        "set to true or false to specify if the GROUNDTRUTH POSE stream need to be include in the slam file.",
                        &this->gt, nullptr));
                this->addParameter(TypedParameter<bool>("acc", "accelerometer",
                        "set to true or false to specify if the ACCELEROMETER stream need to be include in the slam file.",
                        &this->accelerometer, nullptr));
            }

            static image_params_t get_image_params () {
                return fr_image_params;
            }

            // these parameters depend on the particular kinect sensor used
            // return the kinect number (0 for default values)
            DatasetOrigin get_sensor_params(DepthSensor::disparity_params_t & depth_disparity_params,
                                            DepthSensor::disparity_type_t & depth_disparity_type,
                                            CameraSensor::intrinsics_t & rgb_intrinsics,
                                            DepthSensor::intrinsics_t & depth_intrinsics,
                                            CameraSensor::distortion_coefficients_t & rgb_distortion,
                                            DepthSensor::distortion_coefficients_t & depth_distortion,
                                            CameraSensor::distortion_type_t & distortion_type
            ) {
                for (uint32_t i = 0; i < 2; i++) {
                    depth_disparity_params[i] = fr_disparity_params[i];
                }
                depth_disparity_type = fr_disparity_type;

                distortion_type = camera_distortion_type;

                if (input.find("freiburg1") != std::string::npos) {
                    for (uint32_t i = 0; i < 4; i++) {
                        rgb_intrinsics[i] = fr1_intrinsics_rgb[i];
                        depth_intrinsics[i] = fr1_intrinsics_depth[i];
                    }
                    for (uint32_t i = 0; i < 5; i++) {
                        rgb_distortion[i] = fr1_distortion_rgb[i];
                        depth_distortion[i] = fr1_distortion_depth[i];
                    }
                    return DatasetOrigin::Freiburg1;
                }

                if (input.find("freiburg2") != std::string::npos) {
                    for (uint32_t i = 0; i < 4; i++) {
                        rgb_intrinsics[i] = fr2_intrinsics_rgb[i];
                        depth_intrinsics[i] = fr2_intrinsics_depth[i];
                    }
                    for (uint32_t i = 0; i < 5; i++) {
                        rgb_distortion[i] = fr2_distortion_rgb[i];
                        depth_distortion[i] = fr2_distortion_depth[i];
                    }
                    return DatasetOrigin::Freiburg2;
                }

                if (input.find("freiburg3") != std::string::npos) {
                    for (uint32_t i = 0; i < 4; i++) {
                        rgb_intrinsics[i] = fr3_intrinsics_rgb[i];
                        depth_intrinsics[i] = fr3_intrinsics_depth[i];
                    }
                    for (uint32_t i = 0; i < 5; i++) {
                        rgb_distortion[i] = fr3_distortion_rgb[i];
                        depth_distortion[i] = fr3_distortion_depth[i];
                    }
                    return DatasetOrigin::Freiburg3;
                }

                if (input.find("ethi") != std::string::npos) {
                    for (uint32_t i = 0; i < 4; i++) {
                        rgb_intrinsics[i] = ethi_intrinsics_rgb[i];
                        depth_intrinsics[i] = ethi_intrinsics_depth[i];
                    }
                    for (uint32_t i = 0; i < 5; i++) {
                        rgb_distortion[i] = ethi_distortion_rgb[i];
                        depth_distortion[i] = ethi_distortion_depth[i];
                    }
                    return DatasetOrigin::ETHI;
                }

                // use default parameters
                for (uint32_t i = 0; i < 4; i++) {
                    rgb_intrinsics[i] = default_intrinsics_rgb[i];
                    depth_intrinsics[i] = default_intrinsics_depth[i];
                }
                for (uint32_t i = 0; i < 5; i++) {
                    rgb_distortion[i] = default_distortion_rgb[i];
                    depth_distortion[i] = default_distortion_depth[i];
                }
                return DatasetOrigin::Default;
            }

            SLAMFile *GenerateSLAMFile() override;
        };

        /****
         * reader used to process TUM rosbags
         * image and sensor parameters derived from TUMReader
         */
        class TUMROSBAGReader : public TUMReader {
        public:
            typedef struct {
                const std::string world;
                const std::string kinect;
                const std::string camera;
                const std::string rgb;
                const std::string optical;
            } gt_frame_ids_t;

        private:
            // ROS topic associated with each sensor
            const std::string depth_topic = "/camera/depth/image";
            const std::string rgb_topic   = "/camera/rgb/image_color";
            const std::string gt_topic    = "/tf";
            const std::string acc_topic   = "/imu";

            // the ground truth topic (/tf) contains several transformations
            // ground truth is built from the following composition:
            // optical frame -> rgb frame -> camera -> kinect -> world
            const gt_frame_ids_t gt_frame_ids = {
                    "/world",
                    "/kinect",
                    "/openni_camera",
                    "/openni_rgb_frame",
                    "/openni_rgb_optical_frame"
            };

        public :
            explicit TUMROSBAGReader(std::string name) : TUMReader(std::move(name)) {
            }

            SLAMFile * GenerateSLAMFile() override;
        };
    }
}


#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_TUM_H_ */

/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_TUM_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_TUM_H_

#include <ParameterComponent.h>
#include <ParameterManager.h>
#include <Parameters.h>

#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/Sensor.h>
#include "DatasetReader.h"

namespace slambench {

  namespace io {

    class TUMReader : public DatasetReader {
     private:
      // I took those numbers from the TUM dataset paper
      static constexpr CameraSensor::intrinsics_t fr1_intrinsics_rgb = {0.80828125, 1.076041667, 0.4978125, 0.531875};
      static constexpr DepthSensor::intrinsics_t fr1_intrinsics_depth = {0.92359375, 1.229375, 0.5171875, 0.4875};
      static constexpr CameraSensor::intrinsics_t fr2_intrinsics_rgb = {0.81390624, 1.085416667, 0.5079687, 0.52020};
      static constexpr DepthSensor::intrinsics_t fr2_intrinsics_depth = {0.9075, 1.212083333, 0.4825, 0.52708};

      static constexpr DepthSensor::disparity_params_t disparity_params = {0.0002, 0.0};
      static constexpr DepthSensor::disparity_type_t disparity_type = DepthSensor::affine_disparity;

      //// Taken from https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats
      //static constexpr CameraSensor::intrinsics_t fr1_intrinsics_rgb   = { 517.3, 516.5, 318.6, 255.3 };
      //static constexpr DepthSensor::intrinsics_t  fr1_intrinsics_depth = { 591.1, 590.1, 331.0, 234.0 };
      //static constexpr CameraSensor::intrinsics_t fr2_intrinsics_rgb   = { 520.9, 521.0, 325.1, 249.7 };
      //static constexpr DepthSensor::intrinsics_t  fr2_intrinsics_depth = { 580.8, 581.8, 308.8, 253.0 };
      static constexpr CameraSensor::intrinsics_t fr3_intrinsics_rgb   = { 0.8365625, 1.123333, 0.50015625, 0.5158333 };
      static constexpr DepthSensor::intrinsics_t  fr3_intrinsics_depth = { 0.886875,	1.1879166, 0.50734375, 0.521041 };

      static constexpr CameraSensor::intrinsics_t ethl_intrinsics_rgb = {538.7 / 640, 540.7 / 480, 319.2 / 640, 233.6 / 480};
      static constexpr DepthSensor::intrinsics_t  ethl_intrinsics_depth = {538.7 / 640, 540.7 / 480, 319.2 / 640, 233.6 / 480};
      // I took those numbers from ORBSLAM2 examples

      static constexpr float fr1_fps =  30.0 ;
      static constexpr float fr1_bf =  40.0 ;
      static constexpr float fr1_ThDepth =  40.0 ;
      static constexpr float fr1_DepthMapFactor =  5000.0 ;

      static constexpr float fr2_fps =  30.0 ;
      static constexpr float fr2_bf =  40.0 ;
      static constexpr float fr2_ThDepth =  40.0 ;
      static constexpr float fr2_DepthMapFactor =  5208.0 ;

      static constexpr CameraSensor::distortion_coefficients_t fr1_distortion_rgb   = { 0.262383, -0.953104, -0.005358, 0.002628, 1.163314 };
      static constexpr CameraSensor::distortion_coefficients_t fr2_distortion_rgb   = { 0.231222, -0.784899, -0.003257, -0.000105, 0.917205 };
      static constexpr CameraSensor::distortion_coefficients_t fr3_distortion_rgb   = { 0,0,0,0,0 };
      static constexpr CameraSensor::distortion_coefficients_t ethl_distortion_rgb   = { 0,0,0,0,0 };

      static constexpr DepthSensor::distortion_coefficients_t  fr1_distortion_depth = { -0.0410, 0.3286, 0.0087, 0.0051, -0.5643 };
      static constexpr DepthSensor::distortion_coefficients_t  fr2_distortion_depth = { -0.2297, 1.4766, 0.0005, -0.0075, -3.4194 };
      static constexpr DepthSensor::distortion_coefficients_t  fr3_distortion_depth = { 0,0,0,0,0 };
      static constexpr DepthSensor::distortion_coefficients_t  ethl_distortion_depth = { 0,0,0,0,0 };

     public:
      std::string input;
      bool grey = true, rgb = true, depth = true, gt = true, accelerometer = false;

      explicit TUMReader(const std::string& name) : DatasetReader(name) {

        this->addParameter(TypedParameter<std::string>("i", "input-directory",
                                                       "path of the TUM dataset directory",
                                                       &this->input, nullptr));

        this->addParameter(TypedParameter<bool>("grey", "grey",
                                                "set to true or false to specify if the GREY stream "
                                                "need to be include in the slam file.",
                                                &this->grey, nullptr));

        this->addParameter(TypedParameter<bool>("rgb", "rgb",
                                                "set to true or false to specify if the RGB stream "
                                                "need to be include in the slam file.",
                                                &this->rgb, nullptr));

        this->addParameter(TypedParameter<bool>("depth", "depth",
                                                "set to true or false to specify if the DEPTH stream "
                                                "need to be include in the slam file.",
                                                &this->depth, nullptr));

        this->addParameter(TypedParameter<bool>("gt", "gt",
                                                "set to true or false to specify if the GROUNDTRUTH POSE "
                                                "stream need to be include in the slam file.",
                                                &this->gt, nullptr));

        this->addParameter(TypedParameter<bool>("accelerometer", "accelerometer",
                                                "set to true or false to specify if the ACCELEROMETER stream "
                                                "need to be include in the slam file.",
                                                &this->accelerometer, nullptr));
      }

      SLAMFile* GenerateSLAMFile() override;
    };

  }  // namespace io
}  // namespace slambench

#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_TUM_H_ */

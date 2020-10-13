/*

 Copyright (c) 2019 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_UZHFPV_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_UZHFPV_H_

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

    class UZHFPVReader : public DatasetReader {
     private:
      // Parameters taken from indoor_forward calibration file: camchain-..indoor_forward_calib_snapdragon_cam.yaml
      static constexpr CameraSensor::intrinsics_t snapdragon_cam0_intrinsics = {278.66723066149086, 278.48991409740296,
                                                                                319.75221200593535, 241.96858910358173};

      static constexpr CameraSensor::distortion_coefficients_t snapdragon_cam0_distortion = {-0.013721808247486035, 0.020727425669427896,
                                                                                             -0.012786476702685545, 0.0025242267320687625};

      static constexpr CameraSensor::intrinsics_t snapdragon_cam1_intrinsics = {277.61640629770613, 277.63749695723294,
                                                                                314.8944703346039, 236.04310050462587};

      static constexpr CameraSensor::distortion_coefficients_t snapdragon_cam1_distortion = {-0.008456929295619607, 0.011407590938612062,
                                                                                             -0.006951788325762078, 0.0015368127092821786};

      // Parameters taken from indoor_forward calibration file: camchain-..indoor_forward_calib_davis_cam.yaml
      static constexpr CameraSensor::intrinsics_t davis_intrinsics = {172.98992850734132, 172.98303181090185,
                                                                      163.33639726024606, 134.99537889030861};

      static constexpr CameraSensor::distortion_coefficients_t davis_distortion = {-0.027576733308582076, -0.006593578674675004,
                                                                                   0.0008566938165177085, -0.00030899587045247486};

     public:
      std::string input;
      bool imu = true, gt = false, stereo = false, events = false;

      explicit UZHFPVReader(const std::string& name) : DatasetReader(name) {

        this->addParameter(TypedParameter<std::string>("i", "input-directory",
                                                       "path of the UZHFPV dataset directory",
                                                       &this->input,
                                                       nullptr));

        this->addParameter(TypedParameter<bool>("s", "stereo",
                                                "set to true or false to specify if the STEREO "
                                                "stream need to be include in the slam file.",
                                                &this->stereo,
                                                nullptr));

        this->addParameter(TypedParameter<bool>("e", "event",
                                                "set to true or false to specify if the EVENT "
                                                "stream need to be include in the slam file.",
                                                &this->events,
                                                nullptr));

        this->addParameter(TypedParameter<bool>("gt", "gt",
                                                "set to true or false to specify if the GROUNDTRUTH POSE "
                                                "stream need to be include in the slam file.",
                                                &this->gt,
                                                nullptr));

        this->addParameter(TypedParameter<bool>("imu", "imu",
                                                "set to true or false to specify if the IMU "
                                                "stream need to be include in the slam file.",
                                                &this->imu,
                                                nullptr));
      }

      SLAMFile* GenerateSLAMFile() override;
    };

  }  // namespace io
}  // namespace slambench

#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_UZHFPV_H_ */

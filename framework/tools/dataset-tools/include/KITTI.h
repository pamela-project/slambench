/*

 Copyright (c) 2023 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_KITTI_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_KIITI_H_

#include <ParameterComponent.h>
#include <ParameterManager.h>
#include <Parameters.h>

#include <io/sensor/CameraSensor.h>
#include <io/sensor/LidarSensor.h>
#include <io/sensor/IMUSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <fstream>
#include "DatasetReader.h"

namespace slambench {

  namespace io {

    class KITTIReader : public DatasetReader {
     private:
      CameraSensor *rgb_sensor = nullptr;
      CameraSensor *grey_sensor = nullptr;
      LidarSensor *lidar_sensor = nullptr;
      IMUSensor *imu_sensor = nullptr;
      GroundTruthSensor *gt_sensor = nullptr;

      std::ifstream istream;

      void AddSensors(SLAMFile &file);
      bool GetFrame(const std::string &dirname, SLAMFile &file, int frame_no);
      bool AddFrames(const std::string &dirname, SLAMFile &file);

     public:
      static constexpr image_params_t image_params = { 1392, 512, 1, 1.0 };

      std::string input;

      bool grey = true, rgb = true, imu = true, lidar = true, gt = true;

      explicit KITTIReader(const std::string &name) : DatasetReader(name) {
        this->addParameter(TypedParameter<std::string>("i", "input-directory",
                                                       "path of the KITTI dataset directory",
                                                       &this->input, nullptr));

        this->addParameter(TypedParameter<bool>("grey", "grey",
                                                "set to true or false to specify if the GREY "
                                                "stream need to be include in the slam file.",
                                                &this->grey, nullptr));

        this->addParameter(TypedParameter<bool>("rgb", "rgb",
                                                "set to true or false to specify if the RGB "
                                                "stream need to be include in the slam file.",
                                                &this->rgb, nullptr));

        this->addParameter(TypedParameter<bool>("imu", "imu",
                                                "set to true or false to specify if the IMU "
                                                "stream need to be include in the slam file.",
                                                &this->imu, nullptr));

        this->addParameter(TypedParameter<bool>("lidar", "lidar",
                                                "set to true or false to specify if the LIDAR "
                                                "stream need to be include in the slam file.",
                                                &this->lidar, nullptr));

        this->addParameter(TypedParameter<bool>("gt", "gt",
                                                "set to true or false to specify if the "
                                                "GROUNDTRUTH POSE stream need to be include in the slam file.",
                                                &this->gt, nullptr));
      }

      SLAMFile *GenerateSLAMFile() override;
    };

  }  // namespace io
}  // namespace slambench

#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_KITTI_H_ */


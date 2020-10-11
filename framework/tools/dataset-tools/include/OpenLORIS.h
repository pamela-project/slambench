/*

 Copyright (c) 2019 Intel Corp.

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_OPENLORIS_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_OPENLORIS_H_


#include <ParameterManager.h>
#include <ParameterComponent.h>
#include <Parameters.h>

#include <io/sensor/Sensor.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include "DatasetReader.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace slambench {

namespace io {
    const int INF = 99;
    typedef std::pair<int, int> trans_direction;

    Eigen::Matrix4f compute_trans_matrix(const std::string &input_name_1,
                                         const std::string &input_name_2,
                                         const std::string &filename);

class OpenLORISReader :  public DatasetReader {

public :
    std::string input;
    bool color = true, grey = true, depth = true, aligned_depth = true, fisheye1 = true, fisheye2 = true,
        d400_accel = true, d400_gyro = true, t265_accel = true, t265_gyro = true, odom = true, gt = true;

    OpenLORISReader(std::string name) : DatasetReader(name) {

        this->addParameter(TypedParameter<std::string>("i",     "input-directory",       "path of the OpenLORIS dataset directory",   &this->input, nullptr));
        this->addParameter(TypedParameter<bool>("grey",     "grey",       "set to true or false to specify if the GREY stream need to be include in the slam file.",   &this->grey, nullptr));
        this->addParameter(TypedParameter<bool>("color",     "color",       "set to true or false to specify if the RGB stream need to be include in the slam file.",   &this->color, nullptr));
        this->addParameter(TypedParameter<bool>("depth",     "depth",       "set to true or false to specify if the DEPTH stream need to be include in the slam file.",   &this->depth, nullptr));
        this->addParameter(TypedParameter<bool>("aligned_depth",     "aligned_depth",       "set to true or false to specify if the ALIGNED_DEPTH stream need to be include in the slam file.",   &this->aligned_depth, nullptr));
        this->addParameter(TypedParameter<bool>("fisheye1",     "fisheye1",       "set to true or false to specify if the FISHEYE1 stream need to be include in the slam file.",   &this->fisheye1, nullptr));
        this->addParameter(TypedParameter<bool>("fisheye2",     "fisheye2",       "set to true or false to specify if the FISHEYE2 GREY stream need to be include in the slam file.",   &this->fisheye2, nullptr));
        this->addParameter(TypedParameter<bool>("d400_accel",    "d400_accelerometer",       "set to true or false to specify if the D400_ACCELEROMETER stream need to be include in the slam file.",   &this->d400_accel, nullptr));
        this->addParameter(TypedParameter<bool>("d400_gyro",    "d400_gyro",       "set to true or false to specify if the D400_GYRO stream need to be include in the slam file.",   &this->d400_gyro, nullptr));
        this->addParameter(TypedParameter<bool>("t265_accel",    "t265_accelerometer",       "set to true or false to specify if the T265_ACCELEROMETER stream need to be include in the slam file.",   &this->t265_accel, nullptr));
        this->addParameter(TypedParameter<bool>("t265_gyro",    "t265_gyro",       "set to true or false to specify if the T265_GYRO stream need to be include in the slam file.",   &this->t265_gyro, nullptr));
        this->addParameter(TypedParameter<bool>("odom",    "odom",       "set to true or false to specify if the ODOMETRY stream need to be include in the slam file.",   &this->odom, nullptr));
        this->addParameter(TypedParameter<bool>("gt",     "gt",       "set to true or false to specify if the GROUNDTRUTH POSE stream need to be include in the slam file.",   &this->gt, nullptr));

    }

    SLAMFile* GenerateSLAMFile ();

};
}
}



#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_OPENLORIS_H_ */
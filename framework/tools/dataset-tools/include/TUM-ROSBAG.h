/*

 Copyright (c) 2017-2020 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 The development of the interface with ROS datasets (rosbags) is supported
 by the RAIN Hub, which is funded by the Industrial Strategy Challenge Fund,
 part of the UK government’s modern Industrial Strategy. The fund is
 delivered by UK Research and Innovation and managed by EPSRC [EP/R026084/1].

 This code is licensed under the MIT License.

 */


#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_TUM_ROSBAG_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_TUM_ROSBAG_H_


#include "TUM.h"


namespace slambench {
    namespace io {
        class TUMROSBAGReader : public TUMReader {

        public :
            explicit TUMROSBAGReader(std::string name) : TUMReader(std::move(name)) {
            }

            SLAMFile * GenerateSLAMFile() override;
        };
    }
}


#endif /* FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_TUM_ROSBAG_H_ */

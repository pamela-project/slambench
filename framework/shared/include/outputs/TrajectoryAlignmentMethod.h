/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef TRAJECTORYALIGNMENTMETHOD_H
#define TRAJECTORYALIGNMENTMETHOD_H

#include "values/Value.h"

#include <functional>
#include <Eigen/Eigen>

namespace slambench {
    namespace outputs {

        class Output;

        class TrajectoryAlignmentMethod {
        public:
            typedef slambench::values::TrajectoryValue::pose_container_t trajectory_t;
            virtual Eigen::Matrix4f operator()(const trajectory_t &ground_truth, const trajectory_t &trajectory) = 0;
            virtual ~TrajectoryAlignmentMethod() {};
        };

        class OriginalTrajectoryAlignmentMethod : public TrajectoryAlignmentMethod {
        public:
            virtual Eigen::Matrix4f operator()(const trajectory_t &ground_truth, const trajectory_t &trajectory) override;
        };


        class UmeyamaTrajectoryAlignmentMethod : public TrajectoryAlignmentMethod {
        public:
            Eigen::Matrix4f operator()(const trajectory_t &ground_truth, const trajectory_t &trajectory) override;
        };

        class NewTrajectoryAlignmentMethod : public TrajectoryAlignmentMethod {
        public:
            virtual Eigen::Matrix4f operator()(const trajectory_t &ground_truth, const trajectory_t &trajectory) override;
        };
    }
}

#endif /* TRAJECTORYALIGNMENTMETHOD_H */


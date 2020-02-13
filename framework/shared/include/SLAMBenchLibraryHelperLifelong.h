/*

 Copyright (c) 2019 Intel Corp.

 This code is licensed under the MIT License.

 */



#ifndef FRAMEWORK_SHARED_INCLUDE_SLAMBENCHLIBRARYHELPER_LIFELONG_H_
#define FRAMEWORK_SHARED_INCLUDE_SLAMBENCHLIBRARYHELPER_LIFELONG_H_


#include <Parameters.h>
#include <ParameterComponent.h>
#include <io/InputInterface.h>
#include <metrics/MetricManager.h>
#include <outputs/OutputManager.h>
#include <SLAMBenchUI.h>
#include <vector>
#include <Eigen/Core>
#include "SLAMBenchLibraryHelper.h"


class SLAMBenchLibraryHelperLifelong : public SLAMBenchLibraryHelper {


public:
    bool            (* c_sb_relocalize)(SLAMBenchLibraryHelper * );

private:
    SLAMBenchLibraryHelperLifelong ();
    bool sensor_update = false;


public:

    SLAMBenchLibraryHelperLifelong (std::string id, std::string lib, std::ostream& l, slambench::io::InputInterface* i) :
            SLAMBenchLibraryHelper(id, lib, l, i)
	{}
    slambench::outputs::AlignmentOutput* alignment = nullptr;
    void resetSensorUpdate(bool b) {
        sensor_update = b;
    }
    bool getSensorUpdate() {
        return sensor_update;
    }
};

#endif /* FRAMEWORK_SHARED_INCLUDE_SLAMBENCHLIBRARYHELPER_H_ */

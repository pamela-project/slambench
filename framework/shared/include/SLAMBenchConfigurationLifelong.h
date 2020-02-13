/*

 Copyright (c) 2019 Intel Corp.
 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.

 This code is licensed under the MIT License.

 */


#ifndef SLAMBENCH_CONFIGURATION_LIFELONG_H_
#define SLAMBENCH_CONFIGURATION_LIFELONG_H_


#include <SLAMBenchLibraryHelperLifelong.h>
#include "SLAMBenchConfiguration.h"
#include <metrics/ATEMetric.h>
#include <metrics/RPEMetric.h>
#include <outputs/TrajectoryAlignmentMethod.h>
#include <outputs/OutputManagerWriter.h>
#include <metrics/DurationMetric.h>
#include <metrics/PowerMetric.h>
#include "ColumnWriter.h"
#include <SLAMBenchException.h>
#include <sys/time.h>
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <cmath>
#include <unistd.h>
#include <list>


class SLAMBenchConfigurationLifelong : public SLAMBenchConfiguration {
public:
    SLAMBenchConfigurationLifelong();

private :
    slambench::RowNumberColumn row_number;
    slambench::ColumnWriter *cw = nullptr;
    // slambench::metrics::MemoryMetric* memory_metric = nullptr;
	slambench::metrics::DurationMetric* duration_metric = nullptr;
	slambench::metrics::PowerMetric* power_metric = nullptr;

    bool cw_initialised_ = false;
    std::list<slambench::io::InputInterface*> input_interfaces;
    slambench::io::SensorCollection* first_sensors;
    int count = 0;
    bool gt_available;
    std::string gpuInfo;

public :
    bool input_interface_updated = false;
    bool aided_reloc = false;
    std::string alignment_technique_ = "original";
    std::string output_filename_ = "";
    std::vector<std::string> input_filenames;
    void init_cw();
	static void compute_loop_algorithm(SLAMBenchConfiguration* config, bool *stay_on, SLAMBenchUI *ui);
    void InitGroundtruth(bool with_point_cloud = true);
    bool add_input(std::string input_file);
    void add_slam_library(std::string so_file, std::string identifier);
	slambench::io::InputInterface *GetCurrentInputInterface();
	const slambench::io::SensorCollection &GetSensors();
	void AddInputInterface(slambench::io::InputInterface *input_ref);
    void init_sensors();
    void LoadNextInputInterface();
    void OutputToTxt();
};


#endif /* SLAMBENCH_CONFIGURATION_LIFELONG_H_ */

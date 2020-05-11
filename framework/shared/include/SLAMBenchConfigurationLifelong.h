/*

 Copyright (c) 2019 Intel Corp.
 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.

 This code is licensed under the MIT License.

 */


#ifndef SLAMBENCH_CONFIGURATION_LIFELONG_H_
#define SLAMBENCH_CONFIGURATION_LIFELONG_H_


#include <SLAMBenchLibraryHelper.h>
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
#include <memory>


class SLAMBenchConfigurationLifelong : public SLAMBenchConfiguration {
public:
    SLAMBenchConfigurationLifelong();
    void AddLibrary(std::string so_file, std::string identifier);
    bool AddInput(const std::string& input_file);
    void InitSensors();
    void InitWriter();
    void InitGroundtruth(bool with_point_cloud=true);
	static void compute_loop_algorithm(SLAMBenchConfiguration* config, bool *stay_on, SLAMBenchUI *ui);

    std::string alignment_technique_;
    std::string output_filename_;

private:
    slambench::RowNumberColumn row_number_;
    std::unique_ptr<slambench::ColumnWriter> writer_;
    std::shared_ptr<slambench::metrics::Metric> duration_metric_;
    std::shared_ptr<slambench::metrics::Metric> power_metric_;
    std::unique_ptr<slambench::outputs::AlignmentOutput> alignment_ = nullptr;

    std::list<slambench::io::InputInterface*> input_interfaces_;
    slambench::io::SensorCollection* first_sensors_;
    int current_input_id_ = 0;
    bool gt_available_;

    bool input_interface_updated_ = false;
    bool aided_reloc_ = false;
    std::vector<std::string> input_filenames_;

    void InitAlignment();
	slambench::io::InputInterface *GetCurrentInputInterface();
	const slambench::io::SensorCollection &GetSensors();
	void AddInputInterface(slambench::io::InputInterface *input_ref);
    bool LoadNextInputInterface();
    void SaveResults();
};


#endif /* SLAMBENCH_CONFIGURATION_LIFELONG_H_ */

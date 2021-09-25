/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_SHARED_INCLUDE_SLAMBENCHLIBRARYHELPER_H_
#define FRAMEWORK_SHARED_INCLUDE_SLAMBENCHLIBRARYHELPER_H_

#include <Parameters.h>
#include <ParameterComponent.h>
#include <io/InputInterface.h>
#include <metrics/MetricManager.h>
#include <outputs/OutputManager.h>
#include <SLAMBenchUI.h>
#include <utility>
#include <vector>
#include <Eigen/Core>

class SLAMBenchLibraryHelper : public ParameterComponent {

private :
	std::string                        identifier_;
	std::string                        library_name_;
    slambench::metrics::MetricManager  metric_manager_;
    std::ostream&				       log_stream_;
    slambench::io::InputInterface*     input_interface_;
	slambench::outputs::OutputManager  output_manager_;

public:
	bool    (* c_sb_new_slam_configuration)(SLAMBenchLibraryHelper*);
    bool    (* c_sb_init_slam_system)(SLAMBenchLibraryHelper*);
    bool    (* c_sb_update_frame)(SLAMBenchLibraryHelper*, slambench::io::SLAMFrame*);
    bool    (* c_sb_process_once)(SLAMBenchLibraryHelper*);
    bool    (* c_sb_clean_slam_system)();
    bool    (* c_sb_update_outputs)(SLAMBenchLibraryHelper*, const slambench::TimeStamp *ts);
    bool    (* c_sb_relocalize)(SLAMBenchLibraryHelper* );

    SLAMBenchLibraryHelper(const std::string& id,
                           std::string lib,
                           std::ostream& l,
                           slambench::io::InputInterface* i) :
            ParameterComponent(id),
            identifier_(id),
            library_name_(std::move(lib)),
            log_stream_(l),
            input_interface_(i),
            c_sb_new_slam_configuration(nullptr),
            c_sb_init_slam_system(nullptr),
            c_sb_update_frame(nullptr),
            c_sb_process_once(nullptr),
            c_sb_clean_slam_system(nullptr),
            c_sb_update_outputs(nullptr),
            c_sb_relocalize(nullptr)
	{}

    inline const std::string& GetIdentifier() const {return identifier_;}
    inline const std::string& GetLibraryName() const {return library_name_;}
    inline std::ostream& GetLogStream() {return log_stream_;}
    inline slambench::metrics::MetricManager &GetMetricManager() { return metric_manager_; }
    inline slambench::outputs::OutputManager &GetOutputManager() { return output_manager_; }
    inline slambench::io::InputInterface *GetInputInterface() { return input_interface_;}

    inline const slambench::io::SensorCollection &get_sensors() {
        return this->GetInputInterface()->GetSensors();
    }

    inline void update_input_interface(slambench::io::InputInterface* interface)
    {
        input_interface_ = interface;
    }

};
typedef std::vector<SLAMBenchLibraryHelper*> slam_lib_container_t;
#endif /* FRAMEWORK_SHARED_INCLUDE_SLAMBENCHLIBRARYHELPER_H_ */

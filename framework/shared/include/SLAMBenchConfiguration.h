/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef SLAMBENCH_CONFIGURATION_H_
#define SLAMBENCH_CONFIGURATION_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include "ColumnWriter.h"
#include <vector>
#include <memory>
#include <string>
#include <chrono>
#include <list>

#include <ParameterComponent.h>
#include <ParameterManager.h>
#include <SLAMBenchLibraryHelper.h>

#include <io/sensor/SensorCollection.h>
#include <io/InputInterface.h>
#include <dlfcn.h>
#include <io/InputInterfaceManager.h>

#define LOAD_FUNC2HELPER(handle,lib,f)     *(void**)(& lib->f) = dlsym(handle,#f); const char *dlsym_error_##lib##f = dlerror(); if (dlsym_error_##lib##f) {std::cerr << "Cannot load symbol " << #f << dlsym_error_##lib##f << std::endl; dlclose(handle); exit(1);}

static const unsigned int default_frame_limit                  = 0;
static const unsigned int default_start_frame                  = 0;
static const double default_realtime_mult                      = 1;
static const std::string default_dump_volume_file              = "";
static const std::string default_log_file                      = "";
static const std::string default_save_map                      = "";
static const std::vector<std::string> default_slam_libraries   = {};
static const std::vector<std::string> default_input_files      = {};
static const bool                     default_is_false         = false;

typedef std::chrono::time_point<std::chrono::high_resolution_clock> stl_time;

class SLAMBenchConfiguration : public ParameterComponent {
private:
    slam_lib_container_t slam_libs_;
    std::ofstream log_filestream_;
    std::ostream* log_stream_;
    std::string   log_file_;
    std::vector<std::string> input_files_;
    std::vector<std::string> slam_library_names_;
    slambench::RowNumberColumn row_number_;
    std::unique_ptr<slambench::ColumnWriter> writer_;
    std::shared_ptr<slambench::metrics::Metric> duration_metric_;
    std::shared_ptr<slambench::metrics::Metric> power_metric_;
    std::vector<slambench::outputs::AlignmentOutput*> alignments_;
    std::string alignment_technique_ = "umeyama";
    std::string output_filename_;

    std::vector<std::string> input_filenames_;
    slambench::ParameterManager param_manager_;
    slambench::outputs::OutputManager ground_truth_;

    std::vector<std::function<void()>> frame_callbacks_;
    double realtime_mult_;
    int current_input_id_ = 0;
    unsigned int frame_limit_;
    bool initialised_;
    unsigned int start_frame_;
    bool realtime_mode_;
    bool gt_available_;
    bool aided_reloc_ = false;

public:
    SLAMBenchConfiguration(void (*input_callback)(Parameter*, ParameterComponent*) = nullptr,
                           void (*libs_callback)(Parameter*, ParameterComponent*)  = nullptr);
    ~SLAMBenchConfiguration() override;

    void AddFrameCallback(std::function<void()> callback) { frame_callbacks_.push_back(callback); }
    const slam_lib_container_t &GetLoadedLibs() const { return slam_libs_; }
    const slambench::ParameterManager &GetParameterManager() const { return param_manager_; }
    slambench::ParameterManager &GetParameterManager() { return param_manager_; }
    slambench::outputs::OutputManager &GetGroundTruth() { return ground_truth_; }

    /**
     * Initialise the selected libraries and inputs.
     * Initialise the ground truth output manager. All ground truth sensors in
     * the sensor collection are registered as GT outputs, and all frames
     * within the collection are registered as GT output values.
     */
    void InitGroundtruth(bool with_point_cloud = false);
    void InitAlgorithms();
    void InitAlignment();
    void InitWriter();
    void SaveResults();
    void ComputeLoopAlgorithm(bool *stay_on, SLAMBenchUI *ui);
    void AddSLAMLibrary(const std::string& so_file, const std::string &id);
    bool LoadNextInputInterface();
    inline std::ostream& GetLogStream() {
        if (!log_stream_)
            UpdateLogStream();
        return *log_stream_;
    }

    inline void UpdateLogStream() {
        if (log_file_ != "") {
            log_filestream_.open(log_file_.c_str());
            log_stream_ = &log_filestream_;
        } else {
            log_stream_ = &std::cout;
        }
    }
    inline void ResetSensors() {
        param_manager_.ClearComponents();
        for (slambench::io::Sensor *sensor : input_interface_manager_->GetCurrentInputInterface()->GetSensors()) {
            GetParameterManager().AddComponent(dynamic_cast<ParameterComponent*>(&(*sensor)));
        }
    }
    void FireEndOfFrame() { for(auto i : frame_callbacks_) { i(); } }
    void StartStatistics();
    void PrintDse();
    slambench::io::InputInterfaceManager* input_interface_manager_;
};

inline void input_callback(Parameter* param, ParameterComponent* caller) {

    auto config = dynamic_cast<SLAMBenchConfiguration*> (caller);
    assert(config != nullptr && "caller can not be turned into a SLAMBenchConfiguration*");

    auto parameter = dynamic_cast<TypedParameter<std::vector<std::string>>*>(param);
    assert(parameter && "parameter list corrupted");

    config->input_interface_manager_ = new slambench::io::InputInterfaceManager(parameter->getTypedValue());
    //if(!config->input_interface_manager_.initialized()) {
    //    first_sensors_ = &input_ref->GetSensors();
    //} else {
    //    input_ref->GetSensors() = *first_sensors_;
    //}
    for (slambench::io::Sensor *sensor : config->input_interface_manager_->GetCurrentInputInterface()->GetSensors()) {
        config->GetParameterManager().AddComponent(dynamic_cast<ParameterComponent*>(&(*sensor)));
    }
}

inline void help_callback(Parameter*, ParameterComponent* caller) {
    auto config = dynamic_cast<SLAMBenchConfiguration*> (caller);

    std::cerr << " == SLAMBench Configuration ==" << std::endl;
    std::cerr << "  Available parameters :" << std::endl;
    config->GetParameterManager().PrintArguments(std::cerr);

    exit(0);
}

inline void dse_callback(Parameter*, ParameterComponent* caller) {
    auto config = dynamic_cast<SLAMBenchConfiguration*>(caller);
    config->PrintDse();
    exit(0);
}

inline void log_callback(Parameter*, ParameterComponent* caller) {
    auto config = dynamic_cast<SLAMBenchConfiguration*>(caller);
    config->UpdateLogStream();
}

inline void slam_library_callback(Parameter* param, ParameterComponent* caller) {

    auto config = dynamic_cast<SLAMBenchConfiguration*>(caller);
    auto parameter =  dynamic_cast<TypedParameter<std::vector<std::string>>*>(param) ;

    for (auto &library_name : parameter->getTypedValue()) {

        std::string library_filename;
        std::string library_identifier;

        auto pos = library_name.find('=');
        if (pos != std::string::npos)  {
            library_filename   = library_name.substr(0, pos);
            library_identifier = library_name.substr(pos+1);
        } else {
            library_filename = library_name;
        }
        config->AddSLAMLibrary(library_filename, library_identifier);
    }
}
#endif /* SLAMBENCH_CONFIGURATION_H_ */

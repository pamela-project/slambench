/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef SLAMBENCH_CONFIGURATION_H_
#define SLAMBENCH_CONFIGURATION_H_


#include <vector>
#include <string>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <assert.h>

#include <stdexcept>

#include <ParameterComponent.h>
#include <ParameterManager.h>
#include <Parameters.h>

#include <SLAMBenchLibraryHelper.h>

#include <io/SLAMFile.h>
#include <io/sensor/SensorCollection.h>
#include <io/InputInterface.h>


/* Default values */

static const unsigned int default_frame_limit                  = 0;
static const double default_realtime_mult                      = 1;
static const std::string default_dump_volume_file              = "";
static const std::string default_log_file                      = "";
static const std::string default_save_map                      = "";
static const std::vector<std::string> default_slam_libraries   = {};
static const std::vector<std::string> default_input_files      = {};
static const bool                     default_is_false         = false;

/* Classes */


typedef  std::chrono::time_point<std::chrono::high_resolution_clock> stl_time;

class SLAMBenchConfiguration : public ParameterComponent {
public:
    SLAMBenchConfiguration ();
	virtual ~SLAMBenchConfiguration();

private :

    slam_lib_container_t slam_libs;


    std::ofstream log_filestream;
    std::ostream* log_stream;
    std::string   log_file;
    std::string   alignment_technique;
    std::vector<std::string> input_files;
    std::vector<std::string> slam_library_names;
    unsigned int frame_limit;
	
	slambench::io::FrameStream *input_stream_;
    
    slambench::io::InputInterface* input_interface;
	slambench::ParameterManager param_manager_;
	
	slambench::outputs::OutputManager ground_truth_;

	std::vector<std::function<void()>> frame_callbacks_;
	
	bool initialised_;
	bool realtime_mode_;
	double realtime_mult_;
	
	
	
public :

	void AddFrameCallback(std::function<void()> callback) { frame_callbacks_.push_back(callback); }
	
	const slam_lib_container_t &GetLoadedLibs() { return slam_libs; }
    const slambench::ParameterManager &GetParameterManager() const { return param_manager_; }
	slambench::ParameterManager &GetParameterManager() { return param_manager_; }
	
	slambench::outputs::OutputManager &GetGroundTruth() { return ground_truth_; }
	
	/**
	 * Initialise the selected libraries and inputs.
	 * Initialise the ground truth output manager. All ground truth sensors in
	 * the sensor collection are registered as GT outputs, and all frames
	 * within the collection are registered as GT output values.
	 *
	 */
	void InitGroundtruth(bool with_point_cloud = true);

	void InitAlgorithms();
	
	// Clean up data structures used by algorithms
	void CleanAlgorithms();

	
    static void compute_loop_algorithm(SLAMBenchConfiguration * config, bool *stay_on, SLAMBenchUI *ui);

    void add_slam_library    (std::string library_filename, std::string id = "");
    bool add_input (std::string library_filename);


    void                 set_log_file      (std::string f);

public :
	slambench::io::InputInterface *GetInputInterface() {
		if(input_interface == nullptr) {
			throw std::logic_error("Input interface has not been added to SLAM configuration");
		}
		return input_interface;
	}
	const slambench::io::SensorCollection &GetSensors() {

		return this->GetInputInterface()->GetSensors();

	}

	void SetInputInterface(slambench::io::InputInterface *input_ref) {
		input_interface = input_ref;
	}

    inline std::ostream& get_log_stream() {if (!log_stream)  update_log_stream(); return *log_stream;};
    inline void update_log_stream() {

    if (this->log_file != "") {
    	this->log_filestream.open(this->log_file.c_str());
    	this->log_stream = &(this->log_filestream);
    } else {
    	this->log_stream = &std::cout;
    }

    };

	void FireEndOfFrame() { for(auto i : frame_callbacks_) { i(); } }
    void start_statistics   ();


    void print_arguments() ;
    void print_dse();


};





#endif /* SLAMBENCH_CONFIGURATION_H_ */

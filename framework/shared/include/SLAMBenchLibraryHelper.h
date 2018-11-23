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
#include <vector>
#include <Eigen/Core>
#include <SLAMBenchFilterLibraryHelper.h>



#include <dlfcn.h>
#define LOAD_FUNC2HELPER(handle,lib,f)     *(void**)(& lib->f) = dlsym(handle,#f); const char *dlsym_error_##lib##f = dlerror(); if (dlsym_error_##lib##f) {std::cerr << "Cannot load symbol " << #f << dlsym_error_##lib##f << std::endl; dlclose(handle); exit(1);}


static const std::vector<std::string> default_filter_libraries = {};
void filter_library_callback(Parameter* param, ParameterComponent* caller) ;

class SLAMBenchLibraryHelper : public ParameterComponent {

private :
	std::string                        _identifier;
	std::string                        _library_name;
    slambench::metrics::MetricManager  _metric_manager;
    std::ostream&				       _log_stream;
    slambench::io::InputInterface*     _input_interface;
	slambench::outputs::OutputManager  output_manager_;

    std::vector<std::string>           filter_library_names;
    filter_lib_container_t             filter_libs;
public:

public:
	bool            (* c_sb_new_slam_configuration)(SLAMBenchLibraryHelper *) ;
    bool            (* c_sb_init_slam_system)(SLAMBenchLibraryHelper * ) ;
    bool            (* c_sb_update_frame) (SLAMBenchLibraryHelper *, slambench::io::SLAMFrame * ) ;
    bool            (* c_sb_process_once) (SLAMBenchLibraryHelper *) ;
    bool            (* c_sb_clean_slam_system)();
    bool            (* c_sb_update_outputs)(SLAMBenchLibraryHelper *, const slambench::TimeStamp *ts);

private:
    SLAMBenchLibraryHelper ();


public:

    SLAMBenchLibraryHelper (std::string id, std::string lib, std::ostream& l, slambench::io::InputInterface* i) :
    	ParameterComponent(id),
		
    	_identifier(id),
		_library_name(lib),
		_log_stream (l),
		_input_interface (i),
		c_sb_new_slam_configuration(nullptr) ,
		c_sb_init_slam_system(nullptr) ,
		c_sb_update_frame(nullptr) ,
		c_sb_process_once(nullptr) ,
		c_sb_clean_slam_system(nullptr) ,
		c_sb_update_outputs(nullptr)
	{
        this->filter_library_names = {};
        this->addParameter(TypedParameter<std::vector<std::string> >("filter",  "load-filter-library" , "Load a specific Filter library."     , &this->filter_library_names, &default_filter_libraries , filter_library_callback ));
  	}

public :

    inline const std::string& get_identifier() const {return _identifier;};

    inline const std::string& get_library_name() const {return _library_name;};

    inline std::ostream& get_log_stream() {return _log_stream;};

    inline slambench::metrics::MetricManager &GetMetricManager() { return _metric_manager; }
    inline slambench::outputs::OutputManager &GetOutputManager() { return output_manager_; }
	
    inline slambench::io::InputInterface *get_input_interface() {
		if(_input_interface == nullptr) {
			throw std::logic_error("Input interface have not been added to SLAM configuration");
		}
		return _input_interface;
	}

    inline const slambench::io::SensorCollection &get_sensors() {
		return this->get_input_interface()->GetSensors();
	}

    void add_filter_library(std::string so_file , std::string identifier ) ;

    filter_lib_container_t& get_filter_libraries() {
    	return filter_libs;
    }

};

typedef std::vector<SLAMBenchLibraryHelper*>       slam_lib_container_t;



#endif /* FRAMEWORK_SHARED_INCLUDE_SLAMBENCHLIBRARYHELPER_H_ */

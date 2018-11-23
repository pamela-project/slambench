/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef FRAMEWORK_SHARED_INCLUDE_SLAMBenchFilterLibraryHelper_H_
#define FRAMEWORK_SHARED_INCLUDE_SLAMBenchFilterLibraryHelper_H_


#include <Parameters.h>
#include <ParameterComponent.h>
#include <io/InputInterface.h>
#include <metrics/MetricManager.h>
#include <outputs/OutputManager.h>
#include <SLAMBenchUI.h>
#include <vector>
#include <Eigen/Core>
#include <SLAMBenchLibraryHelper.h>



class SLAMBenchFilterLibraryHelper : public ParameterComponent {

private :
	std::string                        _identifier;
	std::string                        _library_name;
    std::ostream&				       _log_stream;
    slambench::io::InputInterface*     _input_interface;

public:
	slambench::io::SLAMFrame *        (* c_sb_filter) (SLAMBenchFilterLibraryHelper *, slambench::io::SLAMFrame * ) ;

private:
    SLAMBenchFilterLibraryHelper ();


public:

    SLAMBenchFilterLibraryHelper (std::string id, std::string lib, std::ostream& l, slambench::io::InputInterface* i) :
    	ParameterComponent(id),
    	_identifier(id),
		_library_name(lib),
		_log_stream (l),
		_input_interface (i),
		c_sb_filter(nullptr)
	{

	}

public :

    inline const std::string& get_identifier() const {return _identifier;};

    inline const std::string& get_library_name() const {return _library_name;};

    inline std::ostream&      get_log_stream() {return _log_stream;};
	
    slambench::io::InputInterface *get_input_interface() {
		if(_input_interface == nullptr) {
			throw std::logic_error("Input interface have not been added to SLAM configuration");
		}
		return _input_interface;
	}

	const slambench::io::SensorCollection &get_sensors() {
		return this->get_input_interface()->GetSensors();
	}



};

typedef std::vector<SLAMBenchFilterLibraryHelper*> filter_lib_container_t;


#endif /* FRAMEWORK_SHARED_INCLUDE_SLAMBenchFilterLibraryHelper_H_ */

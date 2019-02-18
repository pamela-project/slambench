/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "SLAMBenchConfiguration.h"
#include "TimeStamp.h"
#include <Parameters.h>
#include "sb_malloc.h"

#include <io/FrameBufferSource.h>
#include <io/openni2/ONI2FrameStream.h>
#include <io/openni2/ONI2InputInterface.h>

#include <io/openni15/ONI15FrameStream.h>
#include <io/openni15/ONI15InputInterface.h>

#include <io/InputInterface.h>
#include <io/SLAMFrame.h>
#include <io/format/PointCloud.h>
#include <io/sensor/Sensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/PointCloudSensor.h>

#include <metrics/Metric.h>
#include <metrics/ATEMetric.h>
#include <metrics/PowerMetric.h>

#include <values/Value.h>
#include <outputs/Output.h>

#include <boost/optional.hpp>

#include <iostream>
#include <stdexcept>

#include <iomanip>
#include <map>



#include <dlfcn.h>
#define LOAD_FUNC2HELPER(handle,lib,f)     *(void**)(& lib->f) = dlsym(handle,#f); const char *dlsym_error_##lib##f = dlerror(); if (dlsym_error_##lib##f) {std::cerr << "Cannot load symbol " << #f << dlsym_error_##lib##f << std::endl; dlclose(handle); exit(1);}

SLAMBenchConfiguration::~SLAMBenchConfiguration()
{
	CleanAlgorithms();
}

void SLAMBenchConfiguration::add_slam_library(std::string so_file, std::string identifier) {

	std::cerr << "new library name: " << so_file  << std::endl;

	void* handle = dlopen(so_file.c_str(),RTLD_LAZY);

	if (!handle) {
		std::cerr << "Cannot open library: " << dlerror() << std::endl;
		exit(1);
	}

	char *start=(char *)so_file.c_str();
	char *iter = start;
	while(*iter!=0){
		if(*iter=='/')
			start = iter+1;
		iter++;
	}	
	std::string libName=std::string(start);
	libName=libName.substr(3, libName.length()-14);
	SLAMBenchLibraryHelper * lib_ptr = new SLAMBenchLibraryHelper (identifier, libName, this->get_log_stream(),  this->GetInputInterface());
	LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_init_slam_system);
	LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_new_slam_configuration);
	LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_update_frame);
	LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_process_once);
	LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_clean_slam_system);
	LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_update_outputs);
	this->slam_libs.push_back(lib_ptr);


	size_t pre = slambench::memory::MemoryProfile::singleton.GetOverallData().BytesAllocatedAtEndOfFrame;
	if (!lib_ptr->c_sb_new_slam_configuration(lib_ptr)) {
		std::cerr << "Configuration construction failed." << std::endl;
		exit(1);
	}
	size_t post = slambench::memory::MemoryProfile::singleton.GetOverallData().BytesAllocatedAtEndOfFrame;
	std::cerr << "Configuration consumed " << post-pre  << " bytes" << std::endl;

	GetParameterManager().AddComponent(lib_ptr);

	std::cerr << "SLAM library loaded: " << so_file << std::endl;

}

void slam_library_callback(Parameter* param, ParameterComponent* caller) {

	SLAMBenchConfiguration* config = dynamic_cast<SLAMBenchConfiguration*> (caller);

	TypedParameter<std::vector<std::string>>* parameter =  dynamic_cast<TypedParameter<std::vector<std::string>>*>(param) ;

	for (std::string library_name : parameter->getTypedValue()) {


		std::string library_filename   = "";
		std::string library_identifier = "";


		auto pos = library_name.find("=");
		if (pos != std::string::npos)  {
				library_filename   = library_name.substr(0, pos);
				library_identifier = library_name.substr(pos+1);
		} else {
			library_filename = library_name;
		}
		config->add_slam_library(library_filename,library_identifier);
	}
}

void input_callback(Parameter* param, ParameterComponent* caller) {

	SLAMBenchConfiguration* config = dynamic_cast<SLAMBenchConfiguration*> (caller);

	if (!config) {
		std::cerr << "Extremely bad usage of the force..." << std::endl;
		std::cerr << "It happened that a ParameterComponent* can not be turned into a SLAMBenchConfiguration*..." << std::endl;
		exit(1);
	}

	TypedParameter<std::vector<std::string>>* parameter =  dynamic_cast<TypedParameter<std::vector<std::string>>*>(param) ;

	for (std::string input_name : parameter->getTypedValue()) {
		config->add_input(input_name);
	}
}

bool SLAMBenchConfiguration::add_input(std::string input_file) {

	// TODO: Handle other types of interface
	// TODO: Add a getFrameStream in Config to handle that
	// TODO: config will be aware of sensors and then sensors will be able to add there arguments

	if (input_file == "oni2") {
		std::cerr << "Load OpenNI 2 interface ..." << std::endl;
		this->SetInputInterface(new slambench::io::openni2::ONI2InputInterface());
	} else if (input_file == "oni15") {
		std::cerr << "Load OpenNI 1.5 interface ..." << std::endl;
		this->SetInputInterface(new slambench::io::openni15::ONI15InputInterface());
	} else {

		FILE * input_desc = fopen(input_file.c_str(), "r");
		if (input_desc == nullptr) {
			 throw std::logic_error( "Could not open the input file" );
		}
		this->SetInputInterface(new slambench::io::FileStreamInputInterface(input_desc, new slambench::io::SingleFrameBufferSource()));
	}

	for (slambench::io::Sensor *sensor : this->GetInputInterface()->GetSensors()) {
		GetParameterManager().AddComponent(dynamic_cast<ParameterComponent*>(&(*sensor)));
	}

	return true;
}


void help_callback(Parameter* , ParameterComponent* caller) {
	SLAMBenchConfiguration* config = dynamic_cast<SLAMBenchConfiguration*> (caller);
	
	std::cerr << " == SLAMBench Configuration ==" << std::endl;
	config->GetParameterManager().PrintArguments(std::cerr);
	exit(0);
}

void dse_callback(Parameter* , ParameterComponent* caller) {
	SLAMBenchConfiguration* config = dynamic_cast<SLAMBenchConfiguration*> (caller);
	config->print_dse();
	exit(0);
}

void log_callback(Parameter* , ParameterComponent* caller) {
	SLAMBenchConfiguration* config = dynamic_cast<SLAMBenchConfiguration*> (caller);
	config->update_log_stream();
}


void SLAMBenchConfiguration::print_dse () {

    for (SLAMBenchLibraryHelper* lib : this->slam_libs) {

    	std::cout << "libs:"  <<  lib->get_identifier() << "\n" ;

    	for (auto parameter : lib->getParameters()) {
    		std::cout << "argument:" << parameter->getLongOption(lib) << "\n" ;
    		std::cout << parameter->getStrDetails(lib) << "\n" ;

    	}

    }

	exit(0);

}
SLAMBenchConfiguration::SLAMBenchConfiguration () :
    	    	ParameterComponent("") , input_stream_(nullptr)  {

	initialised_ = false;
	this->input_interface = NULL;
	this->log_stream = NULL;
        this->slam_library_names = {};

	// Run Related
	this->addParameter(TypedParameter<unsigned int>("fl",     "frame-limit",      "last frame to compute",                   &this->frame_limit, &default_frame_limit));
	this->addParameter(TypedParameter<std::string>("o",     "log-file",      "Output log file",                   &this->log_file, &default_log_file, log_callback));
	this->addParameter(TypedParameter<std::vector<std::string>>("i",     "input" ,        "Specify the input file or mode." ,  &this->input_files, &default_input_files , input_callback ));
	this->addParameter(TypedParameter<std::vector<std::string> >("load",  "load-slam-library" , "Load a specific SLAM library."     , &this->slam_library_names, &default_slam_libraries , slam_library_callback ));
	this->addParameter(TriggeredParameter("dse",   "dse",    "Output solution space of parameters.",    dse_callback));
	this->addParameter(TriggeredParameter("h",     "help",   "Print the help.", help_callback));
    this->addParameter(TypedParameter<bool>("realtime",     "realtime-mode",      "realtime frame loading mode",                   &this->realtime_mode_, &default_is_false));
    this->addParameter(TypedParameter<double>("realtime-mult",     "realtime-multiplier",      "realtime frame loading mode",                   &this->realtime_mult_, &default_realtime_mult));

	param_manager_.AddComponent(this);

};

void SLAMBenchConfiguration::start_statistics () {

	get_log_stream().setf(std::ios::fixed, std::ios::floatfield);
	get_log_stream().precision(10);

	time_t rawtime;
	struct tm *timeinfo;
	char buffer[80];
	time(&rawtime);
	timeinfo=localtime(&rawtime);
	strftime(buffer,80,"%Y-%m-%d %I:%M:%S",timeinfo);
	this->get_log_stream() << "SLAMBench Report run started:\t" << buffer << std::endl<< std::endl;

	// Print arguments known so far

	this->get_log_stream() << "Properties:" << std::endl<<"=================" << std::endl<< std::endl;

	param_manager_.PrintValues(get_log_stream());
	
}



void SLAMBenchConfiguration::InitGroundtruth(bool with_point_cloud) {

	if(initialised_) {
		return;
	}
	
	if(input_interface != nullptr) {
		auto gt_buffering_stream = new slambench::io::GTBufferingFrameStream(input_interface->GetFrames());
		input_stream_ = gt_buffering_stream;

		if(realtime_mode_) {
			std::cerr << "Real time mode enabled" << std::endl;
			input_stream_ = new slambench::io::RealTimeFrameStream(input_stream_, realtime_mult_, true);
		} else {
			std::cerr << "Process every frame mode enabled" << std::endl;
		}

		GetGroundTruth().LoadGTOutputsFromSLAMFile(input_interface->GetSensors(), gt_buffering_stream->GetGTFrames(), with_point_cloud);
	}
	
	auto gt_trajectory = GetGroundTruth().GetMainOutput(slambench::values::VT_POSE);
	if(gt_trajectory == nullptr) {
		// Warn if there is no ground truth
		std::cerr << "Dataset does not provide a GT trajectory" << std::endl;
	}



	initialised_ = true;
}


void SLAMBenchConfiguration::InitAlgorithms()
{

	assert(this->initialised_);

	for (auto lib : this->slam_libs) {

		//lib->GetMetricManager().BeginInit();
				bool init_worked =  lib->c_sb_init_slam_system(lib) ;
		//lib->GetMetricManager().EndInit();

		if (!init_worked) {
			std::cerr << "Algorithm initialization failed." << std::endl;
			exit(1);
		}

		auto trajectory = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
		if(trajectory == nullptr) {
			std::cerr << "Algo does not provide a main pose output" << std::endl;
			exit(1);
		}
	}

}


void SLAMBenchConfiguration::compute_loop_algorithm(SLAMBenchConfiguration* config, bool *remote_stay_on, SLAMBenchUI *ui) {


	assert(config->initialised_);

	for (auto lib : config->slam_libs) {

		auto trajectory = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
		if(trajectory == nullptr) {
			std::cerr << "Algo does not provide a main pose output" << std::endl;
			exit(1);
		}
	}

	// ********* [[ MAIN LOOP ]] *********

	unsigned int frame_count = 0;

	bool default_true = true;
	bool * stay_on = &default_true;
	bool ongoing = false;
	if (remote_stay_on) stay_on = remote_stay_on;
	
	while(*stay_on) {


		if (ui){
			if(frame_count!=0 &&  !ui->IsFreeRunning() ) {
				if(!ongoing) {
					if(!ui->WaitForFrame() ) {
						std::cerr << "!ui->WaitForFrame() ==> break" << std::endl;
						break;
					}
				}
			}
		}


		// ********* [[ LOAD A NEW FRAME ]] *********

		if(config->input_stream_ == nullptr) {
			std::cerr << "No input loaded." << std::endl;
			break;
		}
		
		slambench::io::SLAMFrame * current_frame = config->input_stream_->GetNextFrame();

		if (current_frame == nullptr) {
			std::cerr << "Last frame processed." << std::endl;
			break;
		}

		if(!*stay_on) {
			std::cerr << "!*stay_on ==> break;" << std::endl;
			break;
		}



		// ********* [[ NEW FRAME PROCESSED BY ALGO ]] *********

		for (auto lib : config->slam_libs) {


			// ********* [[ SEND THE FRAME ]] *********
			ongoing=not lib->c_sb_update_frame(lib,current_frame);
			
			// This algorithm hasn't received enough frames yet.
			if(ongoing) {
				continue;
			}

			// ********* [[ PROCESS ALGO START ]] *********
			lib->GetMetricManager().BeginFrame();
			


			if (not lib->c_sb_process_once (lib)) {
				std::cerr <<"Error after lib->c_sb_process_once." << std::endl;
				exit(1);
			}
			
			slambench::TimeStamp ts = current_frame->Timestamp;
			if(!lib->c_sb_update_outputs(lib, &ts)) {
				std::cerr << "Failed to get outputs" << std::endl;
				exit(1);
			}
			
			lib->GetMetricManager().EndFrame();

		}



		// ********* [[ FINALIZE ]] *********

		current_frame->FreeData();

		
		if(!ongoing) {
			config->FireEndOfFrame();
			if (ui) ui->stepFrame();
			frame_count += 1;
			
			if (config->frame_limit) {
				if (frame_count >= config->frame_limit) {
					break;
				}
			}
		}
		
		
		// we're done with the frame
	}




}

void SLAMBenchConfiguration::CleanAlgorithms()
{
	for (auto lib : slam_libs) {

		std::cerr << "Clean SLAM system ..." << std::endl;
		bool clean_worked = lib->c_sb_clean_slam_system ();


		if (!clean_worked) {
			std::cerr << "Algorithm cleaning failed." << std::endl;
			exit(1);
		} else {
			std::cerr << "Algorithm cleaning succeed." << std::endl;
		}
	}
}

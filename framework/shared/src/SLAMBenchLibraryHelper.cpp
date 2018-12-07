/*
 * SLAMBenchLibraryHelper.cpp
 *
 *  Created on: Nov 23, 2018
 *      Author: toky
 */


#include <SLAMBenchLibraryHelper.h>
#include "sb_malloc.h"



	void SLAMBenchLibraryHelper::add_filter_library(std::string so_file , std::string identifier ) {

		 std::cerr << "new filter library name: " << so_file  << std::endl;

		 void* handle = dlopen(so_file.c_str(),RTLD_LAZY );

		 if (!handle) {
		 	std::cerr << "Cannot open filter library: " << dlerror() << std::endl;
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
		 libName=libName.substr(3, libName.length()-13);

		 SLAMBenchFilterLibraryHelper * lib_ptr = new SLAMBenchFilterLibraryHelper (identifier, libName, this->get_log_stream(),  this->get_input_interface());


		 LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_new_filter_configuration);
		 LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_init_filter);
		 LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_process_filter);

		 size_t pre = slambench::memory::MemoryProfile::singleton.GetOverallData().BytesAllocatedAtEndOfFrame;
		 if (!lib_ptr->c_sb_new_filter_configuration(lib_ptr)) {
			 std::cerr << "Filter configuration construction failed." << std::endl;
			 exit(1);
		 }
		 size_t post = slambench::memory::MemoryProfile::singleton.GetOverallData().BytesAllocatedAtEndOfFrame;
		 std::cerr << "Filter configuration consumed " << post-pre  << " bytes" << std::endl;

		 this->filter_libs.push_back(lib_ptr);
		 this->AddComponent(lib_ptr);

	}

void filter_library_callback(Parameter* param, ParameterComponent* caller) {

	SLAMBenchLibraryHelper* config = dynamic_cast<SLAMBenchLibraryHelper*> (caller);

	if (!config) {
		std::cerr << "Callback wrongly called." << std::endl;
		exit(1);
	}

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
		config->add_filter_library(library_filename,library_identifier);
	}
}


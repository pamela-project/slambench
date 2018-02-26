/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "Parameters.h"
#include "ParameterManager.h"

using namespace slambench;

void ParameterManager::AddComponent(ParameterComponent* component)
{
	components_.push_back(component);
}

void ParameterManager::PrintValues(std::ostream &str) const
{
	for (ParameterComponent*  component_ptr : components_) {
		for (Parameter* param : component_ptr->getParameters()) {
			if (param == nullptr) {
				std::exit(1);
			}
			
			str << param->getLongOption(component_ptr) << ": " << param->getStrValue() << std::endl;
		}
	}
}

void ParameterManager::PrintArguments(std::ostream& str) const
{
	std::string::size_type short_len = 0;
	std::string::size_type long_len  = 0;

	for (ParameterComponent*  component_ptr : components_) {
		for (Parameter* param : component_ptr->getParameters()) {

			std::string short_name = param->getShortOption(component_ptr);
			std::string long_name  = param->getLongOption(component_ptr);

			short_len = std::max (short_name.length(), short_len) ;
			long_len  = std::max (long_name.length(), long_len) ;
		}
	}

	for (ParameterComponent*  component_ptr : components_) {
		std::string comp = component_ptr->getName();
		for (Parameter* param : component_ptr->getParameters()) {

			std::string short_name = param->getShortOption(component_ptr);
			std::string long_name  = param->getLongOption(component_ptr);

			if (short_name.length() > 0) {
				str << "   -"   << std::setw(short_len)  << std::left << short_name ;
			} else {
				str << "    "   << std::setw(short_len)  << std::left << " ";
			}
			str << " --" << std::setw(long_len) << std::left << long_name
													<< " : "
													<< param->getDescription() ;
			std::string defaultstr  = param->getStrDefault();
			std::string currentstr  = param->getStrValue();

			str << " (" ;

			if (defaultstr != "nullptr") {
				str << " Default=" << defaultstr ;
			}
			if (defaultstr != currentstr) {
				str << " Current=" << currentstr ;
			}
			str << " )" << std::endl;
		}
	}
}

bool ParameterManager::ReadArgumentsOrQuit(unsigned int argc, const char* const* const argv, ParameterComponent* callback_data)
{
	if(!ReadArguments(argc, argv, callback_data)) {
		PrintArguments(std::cerr);
		exit(1);
	}
	
	return true;
}


bool ParameterManager::ReadArguments(unsigned int argc, const char* const* const argv, ParameterComponent *callback_data)
{
	std::map<std::string, Parameter*> params_long, params_short;

	// Read and parse argvs
	unsigned i = 1;
	while(i < argc) {

		//******************************************
		// Update_Map
		//******************************************

		params_long.clear();
		params_short.clear();

		// Need to rebuild the parameter map, in case the previous parameter 
		// callback caused new parameters to be added
		for (ParameterComponent*  component_ptr : components_) {
			for (Parameter* param_ptr : component_ptr->getParameters()) {
				if (param_ptr->getLongOption(component_ptr) != "") {

					std::string long_opt = param_ptr->getLongOption(component_ptr);

					if (params_long.count(long_opt)) {
						std::cerr << "*** Duplicated long option replaced: '" << long_opt << "'"<< std::endl;
						exit(1);
						params_long[long_opt] = param_ptr;
					} else {
						params_long[long_opt] = param_ptr;
					}
				}
				if (param_ptr->getShortOption(component_ptr) != "") {

					std::string short_opt = param_ptr->getShortOption(component_ptr);

					if (params_short.count(short_opt)) {
						std::cerr << "*** Duplicated short option replaced: '" << short_opt << "'" << std::endl;
						exit(1);
						params_short[short_opt] = param_ptr;
					} else {
						params_short[short_opt] = param_ptr;
					}
				}

			}
		}


		//******************************************
		// Parse next value
		//******************************************

		const char *the_arg = argv[i];

		Parameter *the_param = nullptr;

		// figure out what kind of arg it is (long or short)
		if(the_arg[0] == '-') {
			if(the_arg[1] == '-') {
				// long arg
				if(params_long.count(&the_arg[2])) {
					the_param = params_long.at(&the_arg[2]);
				} else {
					// error - we've encountered a long param we don't support
					std::cerr << "Unrecognized argument: " << the_arg << std::endl;
					return false;
				}
			} else {
				// short arg
				if(params_short.count(&the_arg[1])) {
					the_param = params_short.at(&the_arg[1]);
				} else {
					std::cerr << "Unrecognized argument: " << the_arg << std::endl;
					return false;
				}
			}

		} else {
			std::cerr << "Error parsing arguments '" << the_arg <<  "'.";
			return false;
		}

		if(the_param) {

			if (the_param->requiresValue()) {
				std::string str_value = argv[i+1] ; i++;
				the_param->setValue(str_value.c_str());
				std::cerr << "Parameter " << the_param->getName() << " assigned value " << the_param->getStrValue() << std::endl;
			} else {
				std::cerr << "Parameter " << the_param->getName() << " triggered." << std::endl;
			}

			if (the_param->getCallback()) {
				(the_param->getCallback())(the_param, callback_data);
			}
		}

		i++;
	}
	
	return true;
}

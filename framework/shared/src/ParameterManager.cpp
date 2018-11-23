/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "Parameters.h"
#include "ParameterManager.h"

using namespace slambench;

void ParameterManager::PrintValues(std::ostream &str, const ParameterComponent* c) const
{
	c = c ? c : this;
	for (const Parameter* param : c->getParameters()) {
		if (param == nullptr) {
			std::exit(1);
		}

		str << param->getLongOption(c) << ": " << param->getStrValue() << std::endl;
	}

	for (ParameterComponent*  component_ptr : c->getComponents()) {
		PrintValues(str, component_ptr);
	}
}



void ParameterManager::PrintArguments(std::ostream& str, const ParameterComponent* c) const
{
	static std::string::size_type short_len = 0;
	static std::string::size_type long_len  = 0;
	static std::string::size_type depth     = 0;

	c = c ? c : this;

	std::string comp = c->getName();

	str << std::endl << std::setw(depth*3)  << std::right << "" << " Component name: " << comp << "Depth:" << depth << std::endl ;

	for (Parameter* param : c->getParameters()) {

		std::string short_name = param->getShortOption(this);
		std::string long_name  = param->getLongOption(this);

		short_len = std::max (short_name.length(), short_len) ;
		long_len  = std::max (long_name.length(), long_len) ;
	}

	if (c->getComponents().size()) {
		str  << std::setw(depth*3)  << std::right << ""  << " Sub components: " << comp << std::endl ;

		depth++;
		for (const ParameterComponent*  component_ptr : c->getComponents()) {
			PrintArguments(str, component_ptr);
		}
		depth--;
	}

	if (c->getParameters().size()) {

		for (Parameter* param : c->getParameters()) {

				std::string short_name = param->getShortOption(c);
				std::string long_name  = param->getLongOption(c);

				str  << std::setw(depth*3)  << std::right << ""  ;

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

bool ParameterManager::ReadArgumentsOrQuit(unsigned int argc, const char* const* const argv)
{
	if(!ReadArguments(argc, argv)) {
		PrintArguments(std::cerr);
		exit(1);
	}
	
	return true;
}



bool ParameterManager::BuildArgumentsList(ParameterComponent *component_ptr) {

	if (!component_ptr) {
		params_long.clear();
		params_short.clear();
		component_ptr = this;
	}

	for (Parameter* param_ptr : component_ptr->getParameters()) {
		if (param_ptr->getLongOption(component_ptr) != "") {

			std::string long_opt = param_ptr->getLongOption(component_ptr);

			if (params_long.count(long_opt)) {
				std::cerr << "*** Duplicated long option replaced: '" << long_opt << "'"<< std::endl;
				return false;
				params_long[long_opt] = param_info_t(component_ptr,param_ptr);
			} else {
				params_long[long_opt] = param_info_t(component_ptr,param_ptr);
			}
		}
		if (param_ptr->getShortOption(component_ptr) != "") {

			std::string short_opt = param_ptr->getShortOption(component_ptr);

			if (params_short.count(short_opt)) {
				std::cerr << "*** Duplicated short option replaced: '" << short_opt << "'" << std::endl;
				return false;
				params_short[short_opt] = param_info_t(component_ptr,param_ptr);
			} else {
				params_short[short_opt] = param_info_t(component_ptr,param_ptr);
			}
		}

	}

	for (ParameterComponent*  sub_component_ptr : component_ptr->getComponents()) {
		this->BuildArgumentsList(sub_component_ptr);
	}


	return true;
}

bool ParameterManager::ReadArguments(unsigned int argc, const char* const* const argv)
{


	// Read and parse argvs
	unsigned i = 1;
	while(i < argc) {

		//******************************************
		// Update_Map
		//******************************************
		this->BuildArgumentsList(nullptr);

		//******************************************
		// Parse next value
		//******************************************

		const char *the_arg = argv[i];

		param_info_t* param_info = nullptr;

		// figure out what kind of arg it is (long or short)
		if(the_arg[0] == '-') {
			if(the_arg[1] == '-') {
				// long arg
				if(params_long.count(&the_arg[2])) {
					param_info = &params_long.at(&the_arg[2]);
				} else {
					// error - we've encountered a long param we don't support
					std::cerr << "Unrecognized argument: " << the_arg << std::endl;
					return false;
				}
			} else {
				// short arg
				if(params_short.count(&the_arg[1])) {
					param_info = &params_short.at(&the_arg[1]);
				} else {
					std::cerr << "Unrecognized argument: " << the_arg << std::endl;
					return false;
				}
			}

		} else {
			std::cerr << "Error parsing arguments '" << the_arg <<  "'.";
			return false;
		}

		auto the_component = param_info->first;
		auto the_param     = param_info->second;
		if(the_param) {

			if (the_param->requiresValue()) {
				std::string str_value = argv[i+1] ; i++;
				the_param->setValue(str_value.c_str());
				std::cerr << "Parameter " << the_param->getName() << " assigned value " << the_param->getStrValue() << std::endl;
			} else {
				std::cerr << "Parameter " << the_param->getName() << " triggered." << std::endl;
			}

			if (the_param->getCallback()) {
				(the_param->getCallback())(the_param, the_component);
			}
		}

		i++;
	}
	
	return true;
}

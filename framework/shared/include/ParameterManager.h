/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef PARAMETERMANAGER_H
#define PARAMETERMANAGER_H

#include <ostream>
#include <vector>
#include <map>
#include <ParameterComponent.h>

typedef std::pair<ParameterComponent*,Parameter*> param_info_t;
namespace slambench {
	class ParameterManager : public ParameterComponent {
	public :
		ParameterManager() :
    	ParameterComponent("") {}
		void ClearComponents() { components_.clear(); };
		void PrintValues(std::ostream &str, const ParameterComponent* c = nullptr) const;
		void PrintArguments(std::ostream &str, const ParameterComponent* c = nullptr) const;
		bool BuildArgumentsList(ParameterComponent *callback_data);
		bool ReadArgumentsOrQuit(unsigned int argc, const char * const * const argv);
		bool ReadArguments(unsigned int argc, const char * const * const argv);

	private:
		std::map<std::string, param_info_t> params_long_, params_short_;
        std::vector<ParameterComponent*> components_;
	};
}

#endif /* PARAMETERMANAGER_H */


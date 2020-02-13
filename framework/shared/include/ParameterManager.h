/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef PARAMETERMANAGER_H
#define PARAMETERMANAGER_H

#include <ostream>
#include <vector>

class ParameterComponent;

namespace slambench {
	class ParameterManager {
	public:
		void AddComponent(ParameterComponent *component);
		void ClearComponents() { components_.clear(); };
		
		void PrintValues(std::ostream &str) const;
		void PrintArguments(std::ostream &str) const;
		
		bool ReadArgumentsOrQuit(unsigned int argc, const char * const * const argv, ParameterComponent *callback_data);
		bool ReadArguments(unsigned int argc, const char * const * const argv, ParameterComponent *callback_data);
	private:
		std::vector<ParameterComponent*> components_;
	};
}

#endif /* PARAMETERMANAGER_H */


/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include <ParameterComponent.h>
#include <Parameters.h>


ParameterComponent::~ParameterComponent() {


		for (Parameter* it : arguments_) {
			delete it;
		}
		arguments_.clear();

}

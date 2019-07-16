/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_SHARED_INCLUDE_PARAMETERCOMPONENT_H_
#define FRAMEWORK_SHARED_INCLUDE_PARAMETERCOMPONENT_H_

#include <string>
#include <vector>


struct Parameter;
class ParameterComponent;

typedef  std::vector<Parameter*>          arguments_vector ;
typedef  std::vector<ParameterComponent*> components_vector ;


class ParameterComponent
{
private:
	std::string        _name;
	arguments_vector   _arguments;
	components_vector  _components;

public:
	ParameterComponent(std::string name) : _name(name) {}

	virtual ~ParameterComponent();

	const arguments_vector &getParameters() const {
		return _arguments;
	}

	const components_vector &getComponents() const {
		return _components;
	}

	std::string getName() const {
		return _name;
	}

	template<typename T>
	void addParameter(T p) {
		T * param_ptr = new T (p);
		this->_arguments.push_back(param_ptr);
		param_ptr->resetValue();


	}

	void AddComponent(ParameterComponent *component) {
		_components.push_back(component);
	}


};



#endif

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
public:
	ParameterComponent(std::string name) : name_(name) {}

	virtual ~ParameterComponent();

	const arguments_vector &getParameters() const {
		return arguments_;
	}

	const components_vector &getComponents() const {
		return components_;
	}

	std::string getName() const {
		return name_;
	}

	template<typename T>
	void addParameter(T p) {
		T * param_ptr = new T (p);
		this->arguments_.push_back(param_ptr);
		param_ptr->resetValue();


	}

	void AddComponent(ParameterComponent *component) {
		components_.push_back(component);
	}

private:
    std::string        name_;
    arguments_vector   arguments_;
    components_vector  components_;

};
#endif

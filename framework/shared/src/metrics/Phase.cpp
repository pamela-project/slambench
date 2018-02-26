/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "metrics/MetricManager.h"
#include "metrics/Phase.h"

using namespace slambench::metrics;

Phase::Phase(MetricManager* mgr, const std::string &phasename) : name_(phasename), manager_(mgr)
{

}

void Phase::Begin()
{
	manager_->BeginPhase(this);
}

void Phase::End()
{
	manager_->EndPhase(this);
}

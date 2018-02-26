/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "sb_malloc.h"
#include "metrics/Metric.h"

#include <cassert>
#include <chrono>
#include <fstream>
#include <string>
#include <unistd.h>
#include <sys/time.h>

using namespace slambench::metrics;

/* Metric Base Class */

Metric::Metric(const std::string& name) : name_(name)
{
}

Metric::~Metric()
{

}

const std::string &Metric::GetDescription() const
{
	static std::string desc = "No description for metric";
	return desc;
}

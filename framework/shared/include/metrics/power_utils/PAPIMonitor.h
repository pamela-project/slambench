/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef FRAMEWORK_INCLUDE_METRICS_POWER_UTILS_PAPI_MONITOR_H_
#define FRAMEWORK_INCLUDE_METRICS_POWER_UTILS_PAPI_MONITOR_H_

#include "values/Value.h"
#include "outputs/Output.h"
#include "PowerMonitor.h"
#include "papi.h"
#include <iostream>


class PAPIMonitor : public PowerMonitor {

public :

	static PAPIMonitor * generate () {

		static PAPIMonitor * loader = nullptr;
		assert (loader == nullptr);

		loader = new PAPIMonitor ();
		if (loader->IsValid()) {
			return loader;
		} else {
			delete loader;
			return nullptr;
		}
	}

private :
	int EventSet;
	long long before_time,after_time;
	long long*values;
	#define MAX_RAPL_EVENTS 64
	char event_names[MAX_RAPL_EVENTS][PAPI_MAX_STR_LEN];
	char units[MAX_RAPL_EVENTS][PAPI_MIN_STR_LEN];

	int num_events=0;

	struct PapiPower {
		double packagePower;
		double pp1Power;
		double dramPower;
		double pp0Power;
	} papiReading;


private :
	slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type current;
	bool init_res   = false;
	bool start_res  = false;

private :

	bool papi_init();
	bool papi_start();
	bool papi_stop();
	bool papi_read();

public :


	PAPIMonitor() : PowerMonitor(), EventSet (PAPI_NULL) {
			init_res = papi_init();
		}


	~PAPIMonitor() {
		if(init_res) {
			papi_stop();
		}
	}

	bool IsValid () {
		return init_res;
	}

	void startSample () {
		start_res = papi_start();
	}

	const slambench::values::ValueDescription &GetType () const {
		static const slambench::values::ValueDescription desc = slambench::values::ValueDescription({
					{"PP0", slambench::values::VT_DOUBLE},
					{"PP1", slambench::values::VT_DOUBLE},
					{"PAPI_PACKAGE", slambench::values::VT_DOUBLE},
					{"PAPI_DRAM", slambench::values::VT_DOUBLE}});
		return desc;
	}

	slambench::values::Value *endSample () {

		assert(start_res);

		bool res = papi_read();

		if (!res) {
			std::cerr << "READ FAILED" << std::endl;
			return new slambench::values::TypeForVT<slambench::values::VT_COLLECTION>::type({});
		}

		auto pp1 = new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(this->papiReading.pp1Power);
		auto pp0 = new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(this->papiReading.pp0Power);
		auto package = new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(this->papiReading.packagePower);
		auto dram = new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(this->papiReading.dramPower);

		return new slambench::values::TypeForVT<slambench::values::VT_COLLECTION>::type({{"PP0", pp0}, {"PP1", pp1}, {"PAPI_PACKAGE", package}, {"PAPI_DRAM", dram}});;

	}
};
#endif /* FRAMEWORK_INCLUDE_METRICS_POWER_UTILS_PAPI_MONITOR_H_ */

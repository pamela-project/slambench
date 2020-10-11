/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_INCLUDE_METRICS_POWER_UTILS_XU3_MONITOR_H_
#define FRAMEWORK_INCLUDE_METRICS_POWER_UTILS_XU3_MONITOR_H_

#include "values/Value.h"
#include "outputs/Output.h"
#include "PowerMonitor.h"
#include <iostream>

enum XU3Sensor {
		SENSOR_A7 = 45, SENSOR_A15 = 40, SENSOR_GPU = 44, SENSOR_DRAM = 41
};

class XU3Monitor : public PowerMonitor {

public :

	static XU3Monitor *generate () {

		static XU3Monitor *loader = nullptr;
		assert (loader == nullptr);

		loader = new XU3Monitor ();
		if (loader->IsValid()) {
			return loader;
		} else {
			delete loader;
			return nullptr;
		}
	}

private :

	FILE *powerA7 = nullptr;
	FILE *powerA15 = nullptr;
	FILE *powerGPU = nullptr;
	FILE *powerDRAM = nullptr;

	double vpowerA7   = 0.0;
	double vpowerA15  = 0.0;
	double vpowerGPU  = 0.0;
	double vpowerDRAM = 0.0;

	double startTime;

private :

	bool odroid_init();
	bool odroid_start();
	bool odroid_sample();
	bool odroid_finish();

	float getPower(XU3Sensor sensor) ;

	bool enableSensor(XU3Sensor sensor) ;

public :


	XU3Monitor() : PowerMonitor() {
		odroid_init();
	}


	~XU3Monitor() {
		if(IsValid ()) {
			odroid_finish();
		}
	}

	bool IsValid () {
		return powerA7 or powerA15 or powerGPU or powerDRAM;
	}

	void startSample () {
		if(IsValid ()) {
		 odroid_start();
		}
	}

	const slambench::values::ValueDescription &GetType () const {
		static const slambench::values::ValueDescription desc = slambench::values::ValueDescription({
					{"A7_Power", slambench::values::VT_DOUBLE},
					{"A15_Power", slambench::values::VT_DOUBLE},
					{"GPU_Power", slambench::values::VT_DOUBLE},
					{"DRAM_Power", slambench::values::VT_DOUBLE}}
		);
		return desc;
	}

	slambench::values::Value *endSample() {

		this->vpowerA7   = 0.0;
		this->vpowerA15  = 0.0;
		this->vpowerGPU  = 0.0;
		this->vpowerDRAM = 0.0;


		odroid_sample();

		auto v1 = new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(this->vpowerA7);
		auto v2 = new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(this->vpowerA15);
		auto v3 = new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(this->vpowerGPU);
		auto v4 = new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(this->vpowerDRAM);

		return new slambench::values::TypeForVT<slambench::values::VT_COLLECTION>::type({
			{"A7_Power", v1}, {"A15_Power", v2},
			{"GPU_Power", v3}, {"DRAM_Power", v4}});;

	}
};
#endif /* FRAMEWORK_INCLUDE_METRICS_POWER_UTILS_PAPI_MONITOR_H_ */

/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifdef PAPI_MONITORING
#include "metrics/power_utils/PAPIMonitor.h"
#endif

#ifdef XU3_MONITORING
#include "metrics/power_utils/XU3Monitor.h"
#endif


#include "metrics/PowerMetric.h"
#include "outputs/TrajectoryAlignmentMethod.h"
#include <boost/optional.hpp>

namespace slambench {
	namespace metrics {

PowerMetric::PowerMetric() : Metric("Power"), last_res(nullptr) {

	pm = nullptr;

#ifdef PAPI_MONITORING
	if (!pm) {
		std::cerr << "*** Test PAPI Monitoring." << std::endl;
		pm = PAPIMonitor::generate();
		if (pm) {std::cerr << "*** PAPI Monitoring works." << std::endl;}
		else {std::cerr << "*** PAPI Monitoring failed." << std::endl;}
	}
#endif

#ifdef XU3_MONITORING
	if (!pm) {
		std::cerr << "*** Test XU3 Monitoring." << std::endl;
		pm = XU3Monitor::generate();
		if (pm) {std::cerr << "*** XU3 Monitoring works." << std::endl;}
		else {std::cerr << "*** XU3 Monitoring failed." << std::endl;}
	}
#endif
	
	if (!pm) {std::cerr << "*** There is no available power monitoring techniques on this system." << std::endl;}

}


const values::ValueDescription &PowerMetric::GetValueDescription() const {
	static const slambench::values::ValueDescription desc =  slambench::values::ValueDescription(slambench::values::VT_COLLECTION);
	if (!pm) 	return  desc;
	return pm->GetType();
}
const std::string& PowerMetric::GetDescription() const {
	static std::string with    = "PAPI Power consumption in Watts.";
	static std::string without = "Power Monitor not found.";
	return pm?with:without;
}

void PowerMetric::MeasureStart(Phase* phase) {
	(void)phase;
	if (pm) {
		pm->startSample();
	}
}

void PowerMetric::MeasureEnd(Phase* phase) {
	(void)phase;
	if (pm) {
		last_res = pm->endSample();
	}
}

slambench::values::Value * PowerMetric::GetValue(Phase* phase) {
	(void)phase;

	if (pm != nullptr) {
		return last_res;
	} else {
		return new slambench::values::TypeForVT<slambench::values::VT_COLLECTION>::type({});
	}
}
	}
}

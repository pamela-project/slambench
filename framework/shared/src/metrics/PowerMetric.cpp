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

PowerMetric::PowerMetric() : Metric("Power"), last_res_(nullptr) {

    pm_ = nullptr;

#ifdef PAPI_MONITORING
	if (!pm_) {
		std::cerr << "*** Test PAPI Monitoring." << std::endl;
		pm_ = PAPIMonitor::generate();
		if (pm_) {std::cerr << "*** PAPI Monitoring works." << std::endl;}
		else {std::cerr << "*** PAPI Monitoring failed." << std::endl;}
	}
#endif

#ifdef XU3_MONITORING
	if (!pm_) {
		std::cerr << "*** Test XU3 Monitoring." << std::endl;
        pm_ = XU3Monitor::generate();
		if (pm_) {std::cerr << "*** XU3 Monitoring works." << std::endl;}
		else {std::cerr << "*** XU3 Monitoring failed." << std::endl;}
	}
#endif
	
	if (!pm_) {std::cerr << "*** There is no available power monitoring techniques on this system." << std::endl;}

}


const values::ValueDescription &PowerMetric::GetValueDescription() const {
	static const slambench::values::ValueDescription desc =  slambench::values::ValueDescription(slambench::values::VT_COLLECTION);
	if (!pm_) 	return  desc;
	return pm_->GetType();
}
const std::string& PowerMetric::GetDescription() const {
	static std::string with    = "PAPI Power consumption in Watts.";
	static std::string without = "Power Monitor not found.";
	return pm_ ? with : without;
}

void PowerMetric::MeasureStart(Phase* phase) {
	(void)phase;
	if (pm_) {
		pm_->startSample();
	}
}

void PowerMetric::MeasureEnd(Phase* phase) {
	(void)phase;
	if (pm_) {
        last_res_ = pm_->endSample();
	}
}

slambench::values::Value * PowerMetric::GetValue(Phase* phase) {
	(void)phase;

	if (pm_ != nullptr) {
		return last_res_;
	} else {
		return new slambench::values::TypeForVT<slambench::values::VT_COLLECTION>::type({});
	}
}
	}
}

/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_SHARED_INCLUDE_METRICS_POWER_UTILS_POWERMONITOR_H_
#define FRAMEWORK_SHARED_INCLUDE_METRICS_POWER_UTILS_POWERMONITOR_H_

class PowerMonitor {
public:
	virtual ~PowerMonitor() {};
	virtual const slambench::values::ValueDescription&  GetType () const = 0 ;
	virtual slambench::values::Value*   endSample() = 0;
	virtual void                       startSample() = 0;

};
#endif /* FRAMEWORK_SHARED_INCLUDE_METRICS_POWER_UTILS_POWERMONITOR_H_ */

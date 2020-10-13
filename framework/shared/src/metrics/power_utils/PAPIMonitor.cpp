/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include <cstring>
#include </usr/include/papi.h>
#include "metrics/power_utils/PAPIMonitor.h"
#include <iostream>

#define DEBUG_INFO(msg) std::cerr << msg << std::endl;
#undef DEBUG_INFO
#define DEBUG_INFO(msg) ;

bool PAPIMonitor::papi_init () {

	int cid,rapl_cid=-1,numcmp;
	int code;
	int r = 0;
	const PAPI_component_info_t *cmpinfo = nullptr;
	PAPI_event_info_t evinfo;

	int retval = PAPI_library_init( PAPI_VER_CURRENT );

	if ( retval != PAPI_VER_CURRENT ) {
		std::cerr << "*** PAPI_library_init failed " << retval << std::endl;
		return false;
	}


	numcmp = PAPI_num_components();

	DEBUG_INFO("Found " << numcmp<< "components");

	for(cid=0; cid<numcmp; cid++) {

		DEBUG_INFO(" - Cmp " << cid << "");

		if ( (cmpinfo = PAPI_get_component_info(cid)) == NULL) {
			std::cerr << "PAPI_get_component_info failed " << std::endl;
			return false;
		}

		DEBUG_INFO("     - Short Name=" << cmpinfo->short_name << "");
		DEBUG_INFO("     - Name=" << cmpinfo->name << "");
		DEBUG_INFO("     - Description=" << cmpinfo->description << "");
		DEBUG_INFO("     - Disabled=" << cmpinfo->disabled << "");

		if (strstr(cmpinfo->name,"rapl")) {

			rapl_cid=cid;

			DEBUG_INFO("     - RAPL FOUND !");

			if (cmpinfo->disabled) {
				std::cerr <<"*** RAPL component disabled: " <<  cmpinfo->disabled_reason << std::endl;
				return false;
			}
			break;
		}
	}

	/* Component not found */
	if (cid==numcmp) {
		std::cerr << "*** PAPI Rapl components not found\n";
		return false;
	}
;
	/* Create EventSet */
	EventSet = PAPI_NULL;
	retval = PAPI_create_eventset( &EventSet );
	if (retval != PAPI_OK) {
		std::cerr << "*** PAPI_create_eventset FAIL "<< std::endl;
		return false;
	}

	/* Add all events */

	code = PAPI_NATIVE_MASK;




	r = PAPI_enum_cmp_event( &code, PAPI_ENUM_FIRST, rapl_cid );


	while ( r == PAPI_OK ) {


		DEBUG_INFO("  - Read event !");

		retval = PAPI_event_code_to_name( code, event_names[num_events] );

		DEBUG_INFO("  - Name = " << event_names[num_events] );
		if ( retval != PAPI_OK ) {
			std::cerr << "Error translating " << code <<  " PAPI_event_code_to_name" << std::endl << retval;
			return false;
		}

		retval = PAPI_get_event_info(code,&evinfo);
		if (retval != PAPI_OK) {
			std::cerr << "Error getting event info " << std::endl << retval;
			return false;
		}

		strncpy(units[num_events],evinfo.units,sizeof(units[0]));
		// buffer must be null terminated to safely use strstr operation on it below
		units[num_events][sizeof(units[0])-1] = '\0';



		retval = PAPI_add_event( EventSet, code );
		if (retval != PAPI_OK) {
			break; /* We've hit an event limit */
		}
		num_events++;

		r = PAPI_enum_cmp_event( &code, PAPI_ENUM_EVENTS, rapl_cid );
	}

	values= (long long int *) calloc(num_events,sizeof(long long));
	if (values==NULL) {
		std::cerr << "Failed to allocate  memory  in PAPI initialisation"<< std::endl;
		return false;
	}
	return true; // That means OK
}
bool PAPIMonitor::papi_start() {

	/* Start Counting */
	before_time=PAPI_get_real_nsec();
	int retval = PAPI_start( EventSet);
	if (retval != PAPI_OK) {
		std::cerr << "ERROR: PAPI_start() " << std::endl<< retval;
	}
	return retval == PAPI_OK;
}
bool PAPIMonitor::papi_read() {

	/* Stop Counting */
	after_time=PAPI_get_real_nsec();
	int retval = PAPI_stop( EventSet, values);
	if (retval != PAPI_OK) {
		std:: cerr <<  "ERROR: PAPI_stop() "<< std::endl<< retval;
		return false;
	}

	DEBUG_INFO("before_time = " << before_time);
	DEBUG_INFO("after_time = " << after_time);
	double elapsed_time=(double)(((double)(after_time-before_time)) / (double)1.0e9);


	DEBUG_INFO("elapsed_time = " << elapsed_time);

	for(int i=0;i<num_events;i++) {

		DEBUG_INFO("Event " << i << ": " << event_names[i] <<  " (" << units[i] << ")")

		if (strstr(units[i],"nJ")) {


			if(strstr(event_names[i],"PACKAGE_ENERGY:PACKAGE0")) {
				DEBUG_INFO("Found PACKAGE.");
				papiReading.packagePower = ((double)values[i]/1.0e9)/elapsed_time;
			}
			if(strstr(event_names[i],"PP1_ENERGY:PACKAGE0")) {
				DEBUG_INFO("Found PP1.");
				papiReading.pp1Power = ((double)values[i]/1.0e9)/elapsed_time;
			}
			if(strstr(event_names[i],"DRAM_ENERGY:PACKAGE0")) {
				DEBUG_INFO("Found DRAM.");
				papiReading.dramPower = ((double)values[i]/1.0e9)/elapsed_time;
			}
			if(strstr(event_names[i],"PP0_ENERGY:PACKAGE0")) {
				DEBUG_INFO("Found PP0.");
				papiReading.pp0Power = ((double)values[i]/1.0e9)/elapsed_time;
			}
		}
	}




	return true;
}
bool PAPIMonitor::papi_stop() {

	/* Done, clean up */
	int retval = PAPI_cleanup_eventset( EventSet );
	if (retval != PAPI_OK) {
		std::cerr << "ERROR: PAPI_cleanup_eventset()" << std::endl<< retval;
		return false;
	}

	retval = PAPI_destroy_eventset( &EventSet );
	if (retval != PAPI_OK) {
		std::cerr << "ERROR: PAPI_destroy_eventset() " << std::endl<< retval;
		return false;
	}


	return true;
}


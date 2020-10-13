/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef TIMINGS_H
#define TIMINGS_H

#ifdef __APPLE__
#include <sys/time.h>       /* clock_t, clock, CLOCKS_PER_SEC */
#else
#include <chrono>
#include <sys/time.h>

#endif


inline float tick() {
    std::chrono::time_point<std::chrono::high_resolution_clock> tickdata;
    tickdata = std::chrono::high_resolution_clock::now();
    static struct timeval t;

    struct timeval diff = t;
    gettimeofday(&t, NULL);

    return ((t.tv_sec - diff.tv_sec) * 1000000u + t.tv_usec - diff.tv_usec)
            / 1.e6;

}

inline double tock() {
#ifdef __APPLE__
		clock_serv_t cclock;
		mach_timespec_t clockData;
		host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
		clock_get_time(cclock, &clockData);
		mach_port_deallocate(mach_task_self(), cclock);
		return (double) clockData.tv_sec + clockData.tv_nsec / 1000000000.0;
#else
		static auto base = std::chrono::high_resolution_clock::now();
		auto end_of_computation = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> total_time =  end_of_computation - base;
		return total_time.count();
#endif
}
//// this is duplicated to fix otherwise missing references in some of the algorithms
//inline double TICK()
//{
//    return tock();
//}
//inline double TOCK(std::string str="", int size=0)
//{
//    return tock();
//}

#endif //TIMINGS_H

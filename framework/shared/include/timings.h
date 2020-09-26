/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef TIMINGS_H
#define TIMINGS_H

#include <chrono>

inline double tock() {
		static auto base = std::chrono::high_resolution_clock::now();
		auto end_of_computation = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> total_time =  end_of_computation - base;
		return total_time.count();
}

#endif //TIMINGS_H

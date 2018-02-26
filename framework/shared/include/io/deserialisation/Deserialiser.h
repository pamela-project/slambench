/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_DESERIALISER_H
#define IO_DESERIALISER_H

#include <cstdio>

namespace slambench {
	namespace io {
		class Deserialiser {
		public:
			Deserialiser(FILE *target_file);
			
			FILE *File();
			
			bool Read(void *target, size_t bytes);
			bool Skip(size_t skip);
			size_t Offset();
			
			bool Good();
		private:
			FILE *_target_file;
		};
	}
}

#endif

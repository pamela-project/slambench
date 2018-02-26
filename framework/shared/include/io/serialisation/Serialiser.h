/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_SERIALISER_H
#define IO_SERIALISER_H

#include <cstdio>

namespace slambench {
	namespace io {
		class Serialiser {
		public:
			Serialiser(FILE *target_file);
			bool Write(const void *data, size_t size);
		protected:
			FILE *File();
			
		private:
			FILE *_file;
		};
	}
}

#endif

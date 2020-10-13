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
			
			FILE *File() const;
			bool Read(void *target, size_t bytes) const;
			bool Skip(size_t skip) const;
			size_t Offset() const;
			bool Good() const;

		private:
			FILE *target_file_;
		};
	}
}
#endif

/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/serialisation/Serialiser.h"

using namespace slambench::io;

Serialiser::Serialiser(FILE *target_file) : _file(target_file) {

}

FILE *Serialiser::File() { 
	return _file; 
}

bool Serialiser::Write(const void *data, size_t size) {
  return fwrite(data, size, 1, File()) == 1;
}


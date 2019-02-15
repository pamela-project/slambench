/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/deserialisation/Deserialiser.h"

using namespace slambench::io;

Deserialiser::Deserialiser(FILE *file) : _target_file(file) {
	
}

FILE *Deserialiser::File() { 
	return _target_file;
}

bool Deserialiser::Read(void *target, size_t bytes) {
	return fread(target, bytes, 1, File()) == 1;
}

bool Deserialiser::Skip(size_t bytes) {
	fseek(File(), bytes, SEEK_CUR);
	return true;
}

size_t Deserialiser::Offset() {
	return ftello(File());
}

bool Deserialiser::Good() {
	return feof(File()) == 0;
}

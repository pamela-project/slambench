/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#include <string>
#include "io/core/Core.h"

using namespace slambench::io::core;

 size_t slambench::io::core::FileSize(FILE *file)
{
	struct stat st;
	fstat(fileno(file), &st);
	return st.st_size;
}

MappedFile::MappedFile(void* mapped_ptr, size_t size) : ptr_(mapped_ptr), size_(size)
{
}

MappedFile::MappedFile(MappedFile&& other) : ptr_(other.ptr_), size_(other.size_)
{
	other.ptr_ = nullptr;
}

MappedFile::~MappedFile()
{
	if (ptr_) {
		munmap(ptr_, size_);
	}
}

MappedFile slambench::io::core::ReadFile(const std::string &filename) 
{
	FILE *file = fopen(filename.c_str(), "r");
	size_t size = FileSize(file);

	void *data = mmap(nullptr, size, PROT_READ, MAP_PRIVATE, fileno(file), 0);
	if(data == MAP_FAILED) {
		throw std::logic_error("");
	}

	fclose(file);

	return MappedFile(data, size);
}

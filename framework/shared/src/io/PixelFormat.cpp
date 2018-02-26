/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/PixelFormat.h"

#include <cassert>
#include <map>

using namespace slambench::io;

static std::map<std::string, pixelformat::EPixelFormat> formats = {
	{"G_I_8", pixelformat::G_I_8},
	{"RGB_III_888", pixelformat::RGB_III_888},
	{"RGBA_IIII_8888", pixelformat::RGBA_IIII_8888},
	{"D_I_8", pixelformat::D_I_8},
	{"D_I_16", pixelformat::D_I_16},
	{"D_F_32", pixelformat::D_F_32},
	{"D_F_64", pixelformat::D_F_64}
};

static std::map<pixelformat::EPixelFormat, std::string> reverse_formats_;
std::map<pixelformat::EPixelFormat, std::string> &GetReverseFormats() {
	if(reverse_formats_.empty()) {
		for(auto i : formats) {
			reverse_formats_[i.second] = i.first;
		}
	}
	return reverse_formats_;
}

pixelformat::EPixelFormat pixelformat::Parse(const std::string& fmt) {
	if(formats.count(fmt)) {
		return formats.at(fmt);
	} else {
		return pixelformat::UNKNOWN;
	}
}

std::string pixelformat::TypeOf(pixelformat::EPixelFormat format) {
	using namespace pixelformat;
	switch(format) {
		case UNKNOWN:
			return "UNKNOWN";
		case G_I_8:
			return "grey";
		case RGB_III_888:
		case RGBA_IIII_8888:
			return "rgb";
		case D_I_8:
		case D_I_16:
		case D_F_32:
		case D_F_64:
			return "depth";
		default:
			return "???";
	}
}

size_t pixelformat::GetPixelSize(pixelformat::EPixelFormat format) {
	using namespace pixelformat;
	switch(format) {
		case G_I_8 : return 1;
		case RGB_III_888 : return 3;
		case RGBA_IIII_8888 : return 4;
		case D_I_8 : return 1;
		case D_F_32 : return 4;
		case D_I_16 : return 2;
		case D_F_64 : return 8;
		
		default:
			assert(false);
			return 0;
	}
}

bool pixelformat::IsRGB(pixelformat::EPixelFormat format) {
	using namespace pixelformat;
	switch(format) {
		case RGB_III_888:
		case RGBA_IIII_8888:
			return true;
		default:
			return false;
	}
}

bool pixelformat::IsGrey(pixelformat::EPixelFormat format) {
	using namespace pixelformat;
	switch(format) {
		case G_I_8:
			return true;
		default:
			return false;
	}
}

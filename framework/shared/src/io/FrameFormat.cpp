/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/FrameFormat.h"

#include <map>

using namespace slambench::io;

static std::map<std::string, frameformat::EFrameFormat> formats = {
	{"raster", frameformat::Raster},
	{"jpeg", frameformat::JPEG},
	{"png", frameformat::PNG}
	
};

frameformat::EFrameFormat frameformat::Parse(const std::string& str) {
	if(formats.count(str)) {
		return formats.at(str);
	}
	return frameformat::UNKNOWN;
}

/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/FrameBuffer.h"
#include "io/SLAMFrame.h"
#include "io/sensor/Sensor.h"
#include "io/sensor/CameraSensor.h"
#include "io/sensor/DepthSensor.h"

#include "lodepng.h"
#include "io/sensor/AccelerometerSensor.h"

#include "io/core/Core.h"

#include <vector>
#include <fstream>
#include <sstream>

#include <cassert>
#include <cmath>
#include <iostream>

using namespace slambench::io;

SLAMFrame::~SLAMFrame() {}

size_t SLAMFrame::GetSize() const {
	if(FrameSensor->IsVariableSize()) {
		return size_if_variable_sized_;
	} else {
		return FrameSensor->GetFrameSize(this);
	}
}

void SLAMFrame::SetVariableSize(uint32_t size)
{
	assert(FrameSensor->IsVariableSize());
	size_if_variable_sized_ = size;
}

uint32_t SLAMFrame::GetVariableSize() const
{
	assert(FrameSensor->IsVariableSize());
	return size_if_variable_sized_;
}


void *SLAMInMemoryFrame::GetData() {
	return Data;
}

void SLAMInMemoryFrame::FreeData() {
	// do nothing
}

SLAMFileFrame::SLAMFileFrame() : _data(nullptr) {}

void *SLAMFileFrame::GetData() {
	if(_data != nullptr) return _data;
	_data = LoadFile();
	
	if(ProcessCallback != nullptr) {
		ProcessCallback(this, (void*)_data);
	}
	
	return _data;
}
void SLAMFileFrame::FreeData() {
	free(_data);
	_data = nullptr;
}

void *TxtFileFrame::LoadFile() {
	// Figure out the underlying data type to load
	const Sensor::sensor_type_t &type = FrameSensor->GetType();
	if(type == CameraSensor::kCameraType || type == DepthSensor::kDepthType) {
		return LoadCameraFile();
	} else {
		return nullptr;
	}
}

template <typename DType> bool ParseElem(const std::string &str, DType &result);

float parse_float(const std::string &str) {
	double multiplier = 0.1;
	// first, find decimal point
	for(auto i : str) {
		if(i == '.' || i == 'e') {
			break;
		}
		multiplier *= 10;
	}
	
	double output = 0;
	for(auto i : str) {
		if(i == 'e') {
			break;
		}
		int digit = 0;
		switch(i) {
			case '0': break;
			case '1': digit = 1; break;
			case '2': digit = 2; break;
			case '3': digit = 3; break;
			case '4': digit = 4; break;
			case '5': digit = 5; break;
			case '6': digit = 6; break;
			case '7': digit = 7; break;
			case '8': digit = 8; break;
			case '9': digit = 9; break;
			case '.': digit = 0; break;
			default:
				throw std::logic_error("unhandled digit");
		}
		output += digit * multiplier;
		multiplier /= 10;
	}
	
	// parse exponent if it exists
	float exponent = 1;
	for(size_t i = 0; i < str.size(); ++i) {
		if(str[i] == 'e') {
			exponent = atoi(str.c_str()+i+1);
		}
	}
	
	// calculate final version
	if(exponent != 1) {
		output = output * pow(10, exponent);
	}
	
	return output;
}

template<> bool ParseElem<float>(const std::string &str, float &result) {
	if(str.size() == 0) return false;
	for(auto c : str) {
		if(!(isdigit(c) || c == '.' || c == 'e' || c == '+')) {
			return false;
		}
	}
	
	result = strtof(str.c_str(), nullptr);
	
	return true;
}

template<> bool ParseElem<double>(const std::string &str, double &result) {
	if(str.size() == 0) return false;
	for(auto c : str) {
		if(!(isdigit(c) || c == '.' || c == 'e' || c == '+')) {
			return false;
		}
	}
	
	result = strtod(str.c_str(), nullptr);
	
	return true;
}

template<> bool ParseElem<unsigned char>(const std::string &str, unsigned char &result) {
	if(str.size() == 0) return false;
	for(auto c : str) {
		if(!isdigit(c)) return false;
	}
	
	result = strtol(str.c_str(), nullptr, 10);
	return true;
}

template <typename DType> std::vector<DType> ParseFile(std::ifstream &str) {
	std::vector<DType> data;
	std::string curr_string;
	
	while(str.good() && !str.eof()) {
		char c = str.get();
		
		if(isdigit(c) || c == '.' || c == 'e' || c == '+') {
			curr_string += c;
		} else {
			DType value;
			if(ParseElem(curr_string, value)) {
				data.push_back(value);
			}
			curr_string = "";
		}
	}
	return data;
}

template <typename DType> void *LoadData(const std::string &filename, int count_per_pxl, int pixels) {
	std::ifstream str (filename.c_str());
	if(!str.good()) {
		throw std::logic_error("Failed to load data file");
	}
	
	DType *data = new DType[pixels * count_per_pxl];
	std::vector<DType> values = ParseFile<DType>(str);
	
	size_t elem_count = count_per_pxl * pixels;
	
	if(values.size() < elem_count) {
		delete [] data;
		std::cerr << "File " << filename << " size mismatch " << values.size() << " < " <<  (elem_count) << std::endl;
		throw std::logic_error("File size mismatch ");
	}
	
	for(size_t i = 0; i < elem_count; ++i) {
		data[i] = values.at(i);
	}
	
	return data;
}

template<pixelformat::EPixelFormat input_format, pixelformat::EPixelFormat output_format> void* ConvertPixels(void *input, size_t pxl_count);

template<> void *ConvertPixels<pixelformat::D_F_32, pixelformat::D_I_16>(void *input, size_t pxl_count) {
	float *pxls = (float*)input;
	uint16_t *outpxls = (uint16_t*)malloc(pxl_count * sizeof(uint16_t));
	
	for(size_t i = 0; i < pxl_count; ++i) {
		outpxls[i] = pxls[i] * 1000;
	}
	
	return outpxls;
}

template<> void *ConvertPixels<pixelformat::D_F_64, pixelformat::D_I_16>(void *input, size_t pxl_count) {
	double *pxls = (double*)input;
	uint16_t *outpxls = (uint16_t*)malloc(pxl_count * sizeof(uint16_t));
	
	for(size_t i = 0; i < pxl_count; ++i) {
		outpxls[i] = pxls[i] * 1000;
	}
	
	return outpxls;
}

void *Convert(void *input, size_t count, pixelformat::EPixelFormat input_format, pixelformat::EPixelFormat output_format) {
	void *output = nullptr;
	
	switch(output_format) {
		case pixelformat::D_I_16:
			switch(input_format) {
				case pixelformat::D_F_32:
					output = ConvertPixels<pixelformat::D_F_32, pixelformat::D_I_16>(input, count);
					break;
				case pixelformat::D_F_64:
					output = ConvertPixels<pixelformat::D_F_64, pixelformat::D_I_16>(input, count);
					break;
				default:
					assert(false);
			}
			break;
		default:
			assert(false);
	}
	
	return output;
}

void *TxtFileFrame::LoadCameraFile() {
	CameraSensor *camera = (CameraSensor*)FrameSensor;
	
	void *data = nullptr;
	switch(InputPixelFormat) {
		case pixelformat::G_I_8:
			data = LoadData<unsigned char>(Filename, 1, camera->Width * camera->Height);
			break;
		case pixelformat::RGB_III_888:
			data = LoadData<unsigned char>(Filename, 3, camera->Width * camera->Height);
			break;
		case pixelformat::D_I_8:
			data = LoadData<unsigned char>(Filename, 1, camera->Width * camera->Height);
			break;
		case pixelformat::D_F_32:
			data = LoadData<float>(Filename, 1, camera->Width * camera->Height);
			break;
		case pixelformat::D_F_64:
			data = LoadData<double>(Filename, 1, camera->Width * camera->Height);
			break;
		case pixelformat::D_I_16:
			data = LoadData<float>(Filename, 1, camera->Width * camera->Height);
			break;
		default:
			assert(false);
	}
	
	if(camera->PixelFormat != InputPixelFormat) {
		void *newdata = Convert(data, camera->Width * camera->Height, InputPixelFormat, camera->PixelFormat);
		free(data);
		data = newdata;
	}

	return data;
}

DeserialisedFrame::DeserialisedFrame(FrameBuffer &buffer, FILE *file) : _buffer(buffer), _file(file) {
	
}

void DeserialisedFrame::SetOffset(size_t new_offset) {
	_offset = new_offset;
}

void *DeserialisedFrame::GetData() {
	_buffer.Acquire();
	_buffer.Reserve(GetSize());
	fseek(_file, _offset, SEEK_SET);
	fread(_buffer.Data(), GetSize(), 1, _file);
	
	return _buffer.Data();
}

void DeserialisedFrame::FreeData() {
	_buffer.Release();
	_buffer.ResetBuffer();
}

void* ImageFileFrame::LoadFile() {
	// get file extension
	std::string ext = Filename.substr(Filename.size()-3, 3);
	
	if(ext == "png") return LoadPng();
	else if(ext == "pgm") return LoadPbm();
	else throw std::logic_error("Unrecognized file type");
}

void* ImageFileFrame::LoadPbm() {
	CameraSensor *camera = nullptr;
	auto &type = FrameSensor->GetType();
	if(type == CameraSensor::kCameraType || type == DepthSensor::kDepthType) {
		camera = (CameraSensor*)FrameSensor;
	} else {
		std::cerr << "Sensor type is " << FrameSensor->GetType() << std::endl;
		throw std::logic_error("Unrecognized sensor type");
	}
	
	std::ifstream file(Filename.c_str());
	std::string mnum, size, max;
	std::getline(file, mnum);
	std::getline(file, size);
	std::getline(file, max);
	
	// only support p5 for now
	if(mnum.compare("P5")) {
		return nullptr;
	}
	
	// don't support max scaling for now
	if(max.compare("255")) {
		return nullptr;
	}
	
	char *orig_ptr = strdup(size.c_str());
	char *ptr = orig_ptr;
	unsigned int size_x = strtol(ptr, &ptr, 10);
	unsigned int size_y = strtol(ptr, &ptr, 10);
	
	if(size_x != camera->Width || size_y != camera->Height) {
		throw std::runtime_error("");
	}
	
	char *data = (char*)malloc(size_x * size_y);
	file.read(data, size_x * size_y);
	
	free(orig_ptr);
	return data;
}


void* ImageFileFrame::LoadPng() {
	Sensor *sensor = nullptr;
	
	auto &type = FrameSensor->GetType();
	if(type == CameraSensor::kCameraType || type == DepthSensor::kDepthType) {
		sensor = (CameraSensor*)FrameSensor;
	} else {
		std::cerr << "Sensor type is " << FrameSensor->GetType() << std::endl;
		throw std::logic_error("Unrecognized sensor type");
	}
	
	// BUG! A DepthSensor is not a CameraSensor!!!!!!!
	//CameraSensor *camera = dynamic_cast<CameraSensor*>(sensor);
	CameraSensor *camera = (CameraSensor*)(sensor);
	if(camera == nullptr) {
		throw std::logic_error("Cannot instantiate an image for something which isn't a camera");
	}
	
	std::vector<unsigned char> pixels;
	unsigned width, height;
	
	auto mappedfile = core::ReadFile(Filename);
	
	
	char *outdata;

	switch(camera->PixelFormat) {

		// Generic case when reading RGB from a PNG file
		case pixelformat::RGB_III_888: {
			if (lodepng::decode(pixels, width, height, (const unsigned char*)mappedfile.Get(), mappedfile.Size(), LCT_RGB, 8)) {
				throw std::logic_error("Failed to decode png");
			}
			if(width != camera->Width || height != camera->Height) {
				throw std::logic_error("PNG width does not match sensor width");
			}
			if(pixels.size() != FrameSensor->GetFrameSize(this)) {
				throw std::logic_error("Decoded png does not match expected frame size");
			}
			
			outdata = (char*)malloc(FrameSensor->GetFrameSize(this));
			memcpy(outdata, pixels.data(), FrameSensor->GetFrameSize(this));

			break;
		}

		// Generic case when reading a Grey scale image from a PNG
		case pixelformat::G_I_8: {
			if (lodepng::decode(pixels, width, height, (const unsigned char*)mappedfile.Get(), mappedfile.Size(), LCT_RGB, 8)) {
				throw std::logic_error("Failed to decode png");
			}
			if(width != camera->Width || height != camera->Height) {
				throw std::logic_error("PNG width does not match sensor width");
			}
			outdata = (char*)malloc(FrameSensor->GetFrameSize(this));

			// convert rgb to greyscale
			for(size_t idx = 0; idx < camera->Width * camera->Height; ++idx) {
				uint32_t total = pixels[3*idx] + pixels[(3*idx) + 1] + pixels[(3*idx) + 2];
				total /= 3;
				outdata[idx] = total;
			}

			break;
		}
		// Special case when reading depth from a TUM PNG
		case pixelformat::D_I_16: {

			lodepng::State state = lodepng::State();
			state.decoder.color_convert = false;

			if (lodepng::decode(pixels, width, height, state, (const unsigned char*)mappedfile.Get(), mappedfile.Size())) {
					throw std::logic_error("Failed to decode png");
			}

			if(width != camera->Width || height != camera->Height) {
				throw std::logic_error("PNG width does not match sensor width");
			}
			
			if(width * height * 2 != pixels.size()) {
				throw std::logic_error("Got an unexpected number of pixels");
			}

			outdata = (char*)malloc(FrameSensor->GetFrameSize(this));

			//Extract the 16bit value, bigendian format (png method for multibyte)
			// Divide by 5: 5000 units = 1m (scene2raw does *1000).
			int k = 0;

			for(unsigned y = 0; y < height; y++) {
				for (unsigned x = 0; x < width; x++) {
					size_t index = y * width * 2 + x * 2;

					int r = pixels[index + 0] * 256 + pixels[index + 1];
					((uint16_t*)outdata)[k++] = r / 5;
				}
			}


			break;
		}
		default:  {
			std::cerr << "Pixel format is " << camera->PixelFormat << std::endl;
			throw std::logic_error("Unsupported pixel format");
			break;
		}
	}

	 if (outdata == nullptr) {
		 throw std::logic_error("Should never happend");
	 }

	return outdata;
}

/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "values/Value.h"
#include "values/ValueDispatch.h"

using namespace slambench::values;

ValueDescription::ValueDescription(ValueType type) : type_(type)
{

}

ValueDescription::ValueDescription(const structured_description& structured_desc) : type_(VT_COLLECTION), structured_type_(structured_desc)
{
	
}

ValueType ValueDescription::GetType() const
{
	return type_;
}

const ValueDescription::structured_description& ValueDescription::GetStructureDescription() const
{
	if(GetType() != VT_COLLECTION) {
		throw std::logic_error("");
	}
	return structured_type_;
}


Value::~Value()
{

}


void Value::Dispatch(ValueDispatch* vd)
{
	switch(GetType()) {
#define HANDLE(vt) case vt: vd->Dispatch((TypeForVT<vt>::type*)this); return;
		
		HANDLE(VT_COLLECTION);
		HANDLE(VT_U64);
		HANDLE(VT_DOUBLE);
		HANDLE(VT_STRING);
		HANDLE(VT_TRAJECTORY);
		HANDLE(VT_POSE);
		HANDLE(VT_POINTCLOUD);
//		HANDLE(VT_FEATURE);
//		HANDLE(VT_FEATURELIST);
//		HANDLE(VT_FRAME);
		HANDLE(VT_MATRIX);
#undef HANDLE
		case VT_UNKNOWN:
		default:
			assert(false);
	}
}

void Value::Dispatch(ConstValueDispatch* vd) const
{
	switch(GetType()) {
#define HANDLE(vt) case vt: vd->Dispatch((const TypeForVT<vt>::type*)this); return;
		
		HANDLE(VT_COLLECTION);
		HANDLE(VT_U64);
		HANDLE(VT_DOUBLE);
		HANDLE(VT_STRING);
		HANDLE(VT_TRAJECTORY);
		HANDLE(VT_POSE);
		HANDLE(VT_POINTCLOUD);
//		HANDLE(VT_FEATURE);
//		HANDLE(VT_FEATURELIST);
//		HANDLE(VT_FRAME);
		HANDLE(VT_MATRIX);
#undef HANDLE
		case VT_UNKNOWN:
		default:
			assert(false);
	}
}


FrameValue::FrameValue(const FrameValue& other) : Value(VT_FRAME), width_(other.width_), height_(other.height_), pxl_format_(other.pxl_format_), data_(other.data_) 
{

}

FrameValue::FrameValue(uint32_t width, uint32_t height, slambench::io::pixelformat::EPixelFormat pxl_format, void* data) : Value(VT_FRAME), width_(width), height_(height), pxl_format_(pxl_format)
{
	auto depth = slambench::io::pixelformat::GetPixelSize(pxl_format_);
	
	size_t datasize = width * height * depth;
	data_.resize(datasize);
	memcpy(data_.data(), data, datasize);
}

FrameValue::FrameValue(uint32_t width, uint32_t height, slambench::io::pixelformat::EPixelFormat pxl_format) : Value(VT_FRAME), width_(width), height_(height), pxl_format_(pxl_format)
{
	auto depth = slambench::io::pixelformat::GetPixelSize(pxl_format_);
	
	size_t datasize = width * height * depth;
	data_.resize(datasize);
}

ValueCollectionValue::~ValueCollectionValue() {
	for(auto i : values_) {
		delete i.second;
	}
}

ValueListValue::~ValueListValue() 
{
	for(auto i : values_) {
		delete i;
	}
}

FeatureValue::FeatureValue(const Eigen::Matrix4f& pose, const FrameValue& patch) : Value(VT_FEATURE), image_patch_(patch), pose_(pose)
{

}

FeatureValue::~FeatureValue() {
	
}

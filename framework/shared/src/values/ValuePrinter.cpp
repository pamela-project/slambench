/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "values/ValuePrinter.h"

using namespace slambench::values;

ValuePrinter::ValuePrinter(std::ostream& str) : str_(str)
{

}

ValuePrinter::~ValuePrinter()
{

}

void ValuePrinter::Dispatch(const TypeForVT<VT_U64>::type* value)
{
	str_ << value->GetValue();
}
void ValuePrinter::Dispatch(const TypeForVT<VT_DOUBLE>::type* value)
{
	str_ << value->GetValue();
}
void ValuePrinter::Dispatch(const TypeForVT<VT_STRING>::type* value)
{
	str_ << value->GetValue();
}
void ValuePrinter::Dispatch(const TypeForVT<VT_POINTCLOUD>::type* value)
{
	(void)value;
	str_ << "(point cloud)";
}
void ValuePrinter::Dispatch(const TypeForVT<VT_COLLECTION>::type* value)
{
	(void)value;
	str_ << "(collection)";
}
void ValuePrinter::Dispatch(const TypeForVT<VT_TRAJECTORY>::type* value)
{
	(void)value;
	str_ << "(trajectory)";
}
void ValuePrinter::Dispatch(const TypeForVT<VT_POSE>::type* value)
{
	// extract x, y, z
	auto &matrix = value->GetValue();
	auto x = matrix(0, 3);
	auto y = matrix(1, 3);
	auto z = matrix(2, 3);
	str_ << x << ", " << y << ", " << z;
}
void ValuePrinter::Dispatch(const TypeForVT<VT_MATRIX>::type* value)
{
	str_ << value->GetValue();
}

/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "ColumnWriter.h"

#include "metrics/Metric.h"
#include "outputs/Output.h"
#include "values/ValueInterface.h"
#include "values/ValuePrinter.h"

using namespace slambench;

std::string LibColumnInterface::GetHeaderPrefix() const
{
	std::string libname = GetLib()->getName();
	if(libname != "") {
		if (libname.find("-") != libname.npos) {
			throw std::logic_error("This library name contains an illegal character '-' !" );
		}
		libname = libname + "-";
	}
	return libname;
}

ValueLibColumnInterface::ValueLibColumnInterface(SLAMBenchLibraryHelper* lib, outputs::BaseOutput* output) : ValueLibColumnInterface(lib, new values::OutputValueInterface(*output), output->GetName()) {}

ValueLibColumnInterface::ValueLibColumnInterface(SLAMBenchLibraryHelper *lib, metrics::Metric *metric, metrics::Phase *phase) : ValueLibColumnInterface(lib, new values::MetricValueInterface(lib->GetMetricManager(), *metric, *phase), metric->GetName() + "_" + phase->GetName()) {}

void ValueLibColumnInterface::Write(std::ostream& str)
{
	values::ValuePrinter printer(str);
	auto value = value_->Get();
	if(value != nullptr) {
		value_->Get()->Dispatch(&printer);
	} else {
		throw std::logic_error("Could not print missing value");
	}
}

void ValueLibColumnInterface::WriteHeader(std::ostream& str) const
{
	str << GetHeaderPrefix() << name_;
}

CollectionValueLibColumnInterface::CollectionValueLibColumnInterface(SLAMBenchLibraryHelper* lib, metrics::Metric* metric, metrics::Phase* phase) : ValueLibColumnInterface(lib,metric, phase) {}

CollectionValueLibColumnInterface::CollectionValueLibColumnInterface(SLAMBenchLibraryHelper* lib, outputs::BaseOutput* output) : ValueLibColumnInterface(lib, output) {}

void CollectionValueLibColumnInterface::WriteHeader(std::ostream& str) const
{
	auto &spec = GetValueInterface()->GetDescription();
	if(spec.GetType() != values::VT_COLLECTION) {
		throw std::logic_error("");
	}
	if(spec.GetStructureDescription().empty()) {
		return;
	}
	
	auto i = spec.GetStructureDescription().begin();
	str << GetHeaderPrefix() << i->first;
	i++;
	while(i != spec.GetStructureDescription().end()) {
		str << "\t" << GetHeaderPrefix() << i->first;
		i++;
	}
}

void CollectionValueLibColumnInterface::Write(std::ostream& str)
{
	values::ValuePrinter printer(str);
 	auto value = (values::TypeForVT<values::VT_COLLECTION>::type*)GetValueInterface()->Get();

	if(value->GetValue().empty()) {
		WriteInvalid(str);
	} else {
		auto i = value->GetValue().begin();
		i->second->Dispatch(&printer);
		i++;
		while(i != value->GetValue().end()) {
			str << "\t";
			i->second->Dispatch(&printer);
			i++;
		}
	}
}

void CollectionValueLibColumnInterface::WriteInvalid(std::ostream& str)
{
	auto &spec = GetValueInterface()->GetDescription().GetStructureDescription();
	if(spec.empty()) {
		return;
	}
	
	auto i = spec.begin();
	str << "(n/a)";
	i++;
	while(i != spec.end()) {
		str << "\t(n/a)";
		i++;
	}
}

void OutputTimestampColumnInterface::Write(std::ostream& str)
{
	TimeStamp ts = output_->GetMostRecentValue().first;
	double timestamp = ts.S + (ts.Ns/1000000000.0);
	
	str << timestamp;
}

void OutputTimestampColumnInterface::WriteHeader(std::ostream& str) const
{
	str << "Timestamp";
}

RowNumberColumn::RowNumberColumn() : counter_(1) {}

void RowNumberColumn::Write(std::ostream& str)
{
	str << counter_;
	counter_++;
}

void RowNumberColumn::WriteHeader(std::ostream& str) const
{
	str << "Frame Number";
}


ColumnWriter::ColumnWriter(std::ostream& str, const std::string &separator) : str_(str), separator_(separator) {}

void ColumnWriter::AddColumn(ColumnInterface* interface)
{
	columns_.push_back(interface);
}

void ColumnWriter::PrintRow()
{
	if(!columns_.empty()) {
		auto i = columns_.begin();
		columns_.front()->Write(str_);
		i++;
		
		while(i != columns_.end()) {
			str_ << separator_;
			(*i)->Write(str_);
			i++;
		}
	}
	
	str_ << std::endl;	
}

void ColumnWriter::PrintHeader()
{
	str_  <<  std::endl ;
	str_  << "Statistics:" << std::endl<<"=================" << std::endl<< std::endl;
	if(!columns_.empty()) {
		auto i = columns_.begin();
		columns_.front()->WriteHeader(str_);
		i++;
		
		while(i != columns_.end()) {
			str_ << separator_;
			(*i)->WriteHeader(str_);
			i++;
		}
	}
	
	str_ << std::endl;	
}

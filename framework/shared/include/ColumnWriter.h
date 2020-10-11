/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef COLUMNWRITER_H
#define COLUMNWRITER_H

#include <ostream>
#include <string>
#include <vector>

class SLAMBenchLibraryHelper;


namespace slambench {

	namespace values {
	class ValueInterface;
	}
	namespace outputs {
		class BaseOutput;
	}

	namespace metrics {
		class Metric;
		class Phase;
	}

}

namespace slambench {

	class ColumnInterface {
	public:
		virtual ~ColumnInterface() = default;
		virtual void WriteHeader(std::ostream &str) const = 0;
		virtual void Write(std::ostream &str) = 0;
	};
	
	class LibColumnInterface : public ColumnInterface {
	public:
		LibColumnInterface(SLAMBenchLibraryHelper *lib) : lib_(lib) {}
		virtual ~LibColumnInterface() = default;
		void Write(std::ostream& str) override = 0;
		void WriteHeader(std::ostream& str) const override = 0;

	protected:
		SLAMBenchLibraryHelper *GetLib() { return lib_; }
		const SLAMBenchLibraryHelper *GetLib() const { return lib_; }
		
		std::string GetHeaderPrefix() const;
	private:
		SLAMBenchLibraryHelper *lib_;
	};
	
	class ValueLibColumnInterface : public LibColumnInterface {
	public:
		ValueLibColumnInterface(SLAMBenchLibraryHelper *lib, metrics::Metric *metric, metrics::Phase *phase);
		ValueLibColumnInterface(SLAMBenchLibraryHelper *lib, outputs::BaseOutput *output);
		~ValueLibColumnInterface() override = default;
		
		virtual void Write(std::ostream& str) override;
		virtual void WriteHeader(std::ostream& str) const override;

	protected:
		const std::string &GetName() { return name_; }
		values::ValueInterface *GetValueInterface() { return value_; }
		const values::ValueInterface *GetValueInterface() const { return value_; }
		
	private:
		ValueLibColumnInterface(SLAMBenchLibraryHelper *lib, values::ValueInterface *value, const std::string &name) : LibColumnInterface(lib), value_(value), name_(name) {}
		values::ValueInterface *value_;
		const std::string name_;
	};
	
	class CollectionValueLibColumnInterface : public ValueLibColumnInterface {
	public:
		
		CollectionValueLibColumnInterface(SLAMBenchLibraryHelper *lib, metrics::Metric *metric, metrics::Phase *phasespec);
		CollectionValueLibColumnInterface(SLAMBenchLibraryHelper *lib, outputs::BaseOutput *output);
		
		void Write(std::ostream& str) override;
		void WriteHeader(std::ostream& str) const override;
		
	private:
		void WriteInvalid(std::ostream &str);
	};
	
	class OutputTimestampColumnInterface : public ColumnInterface {
	public:
		explicit OutputTimestampColumnInterface(outputs::BaseOutput *output) : output_(output) {};
		~OutputTimestampColumnInterface() override = default;
		void Write(std::ostream& str) override;
		void WriteHeader(std::ostream& str) const override;
	private:
		outputs::BaseOutput *output_;
	};
		
	class RowNumberColumn : public ColumnInterface {
	public:
		RowNumberColumn();
		
		void Write(std::ostream& str) override;
		void WriteHeader(std::ostream& str) const override;

	private:
		uint64_t counter_;
	};
	
	class ColumnWriter {
	public:
		ColumnWriter(std::ostream &str, const std::string &separator);
		~ColumnWriter() = default;

		void AddColumn(ColumnInterface *interface);
		void PrintRow();
		void PrintHeader();
	private:
		std::vector<ColumnInterface*> columns_;
		std::ostream &str_;
		std::string separator_;
	};
}

#endif /* COLUMNWRITER_H */


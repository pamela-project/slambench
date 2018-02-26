/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef METRICVALUE_H
#define METRICVALUE_H

#include <cassert>
#include <cstring>
#include <cstdint>
#include <vector>

namespace slambench {
	namespace metrics {
		class MetricValue {
		private:
			MetricValue(size_t size, void *data) { data_.resize(size); memcpy(data_.data(), data, size); }
		public:
			MetricValue(const MetricValue &other) : data_(other.data_) {}
			
			template<typename T> MetricValue(T value) : MetricValue(sizeof(value), &value) {}
			
			const void *GetStorage() const { return (void*)data_.data(); }
			template<typename T> T As() const { assert(sizeof(T) == data_.size()); return *(T*)GetStorage(); }
		private:
			std::vector<char> data_;
		};
	}
}

#endif /* METRICVALUE_H */


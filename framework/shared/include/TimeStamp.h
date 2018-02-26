/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <chrono>
#include <cassert>
#include <cstdint>
#include <ostream>
#include <thread>

namespace slambench {
	struct TimeStamp {
	public:
		uint32_t S;
		uint32_t Ns;
		
		uint64_t ToNs() const { return (S * 1000000000ULL) + Ns; }
		double ToS() const { return ToNs() / 1000000000.0; }
		
		static TimeStamp get(uint32_t S, uint32_t Ns) {
			assert(Ns < 1000000000);
			TimeStamp ts;
			ts.S = S;
			ts.Ns = Ns;
			return ts;
		}
		
		static TimeStamp FromNs(uint64_t ns) {
			TimeStamp ts;
			ts.S = ns / 1000000000ULL;
			ts.Ns = ns % 1000000000ULL;
			return ts;
		}
		
		static TimeStamp Now() 
		{ 
			auto time = std::chrono::duration_cast<std::chrono::duration<uint64_t,std::ratio<1,1000000000>>>(std::chrono::high_resolution_clock::now().time_since_epoch());
			return get(time.count() / 1000000000, time.count() % 1000000000);
		}
	};
	
	typedef std::chrono::duration<uint64_t, std::ratio<1, 1000000000>> Duration;
	
	inline void Sleep(const Duration d) {
		std::this_thread::sleep_for(d);
	}
	
	inline bool operator==(const TimeStamp &left, const TimeStamp &right) {
		return left.S == right.S && left.Ns == right.Ns;
	}
	
	inline bool operator!=(const TimeStamp &left, const TimeStamp &right) {
		return left.S != right.S || left.Ns != right.Ns;
	}
	
	inline bool operator<(const TimeStamp& left, const TimeStamp& right) {
		return ( left.S < right.S ) or ( left.S == right.S and left.Ns < right.Ns );
	}

	inline bool operator<=(const TimeStamp& left, const TimeStamp& right) {
		return ( left.S < right.S ) or ( left.S == right.S and left.Ns <= right.Ns );
	}

	inline bool operator>(const TimeStamp& left, const TimeStamp& right) {
		return ( left.S > right.S ) or ( left.S == right.S and left.Ns > right.Ns );
	}
	
	inline bool operator>=(const TimeStamp& left, const TimeStamp& right) {
		return !(left < right);
	}

	inline Duration operator-(const TimeStamp& left, const TimeStamp& right) {
		uint64_t total_ns_left = left.Ns + (left.S * 1000000000ULL);
		uint64_t total_ns_right = right.Ns + (right.S * 1000000000ULL);
		
		uint64_t ns_diff = total_ns_left - total_ns_right;
		
		return Duration(ns_diff);
	}
	
	inline TimeStamp operator*(const TimeStamp &left, double right) {
		uint64_t total_ns = left.ToNs();
		total_ns *= right;
		
		return TimeStamp::FromNs(total_ns);
	}
	
	inline void operator*=(TimeStamp &left, double right) {
		uint64_t total_ns = left.ToNs();
		total_ns *= right;
		
		left = TimeStamp::FromNs(total_ns);
	}
	
	inline TimeStamp operator+(const TimeStamp &left, const TimeStamp &right) {
		TimeStamp res;
		res.S = left.S + right.S;
		res.Ns = left.Ns + right.Ns;
		if(res.Ns >= 1000000000) {
			res.Ns -= 1000000000;
			res.S += 1;
		}
		return res;
	}
	
	inline std::ostream &operator<<(std::ostream &str, const TimeStamp &t) {
		double s = t.S + (t.Ns / 1000000000.0);
		str << std::to_string(s);
		return str;
	}
}

#endif /* TIMESTAMP_H */


/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef FRAMEWORK_SHARED_INCLUDE_UTILS_H_
#define FRAMEWORK_SHARED_INCLUDE_UTILS_H_

#include <algorithm>
#include <atomic>
#include <string>

inline std::string getFileExt(const std::string& s) {

   std::string str = "";
   size_t i = s.rfind('.', s.length());
   if (i != std::string::npos) {
      str = (s.substr(i+1, s.length() - i));
      std::transform(str.begin(), str.end(),str.begin(), ::toupper);
   }
   return str;
}

class FastLock {
public:
	FastLock() { flag_.clear(); }
	void lock() { while(flag_.test_and_set()) ; }
	void unlock() { flag_.clear(); }
	
private:
	std::atomic_flag flag_;
};

#endif /* FRAMEWORK_SHARED_INCLUDE_UTILS_H_ */

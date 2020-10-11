/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_SHARED_INCLUDE_SLAMBENCHEXCEPTION_H_
#define FRAMEWORK_SHARED_INCLUDE_SLAMBENCHEXCEPTION_H_

#include <exception>
#include <string>

class SLAMBenchException : public std::exception {
  private:
    std::string err_msg_;

  public:
    SLAMBenchException(const char *msg) : err_msg_(msg) {};
    ~SLAMBenchException() throw() {};
    const char *what() const throw() { return this->err_msg_.c_str(); };
};


#endif /* FRAMEWORK_SHARED_INCLUDE_SLAMBENCHEXCEPTION_H_ */

/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef FRAMEWORK_SHARED_INCLUDE_PARAMETERS_H_
#define FRAMEWORK_SHARED_INCLUDE_PARAMETERS_H_

#include <math_types.h>
#include <utils.h>

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <map>
#include <iomanip>
#include <vector>
#include <typeinfo>

#include "ParameterComponent.h"


template <typename T>
std::string precise_to_string(T value)
{
    std::ostringstream os ;
    os  << std::setprecision(std::numeric_limits<T>::digits10 + 1) << value ;
    return os.str() ;
}



inline std::ostream & operator<<(std::ostream & out, const std::vector<int> & m) {
	for (auto it = m.begin() ; it != m.end() ; it++) {
		if (it != m.begin()) out << ",";
		out << *it;
	}
    return out;
}

inline std::ostream & operator<<(std::ostream & out, const sb_uint2 & m) {
    return out << "" <<  m.x << "," << m.y   <<  "";
}

inline std::ostream & operator<<(std::ostream & out, const sb_uint3 & m) {
    return out << "" <<  m.x << "," << m.y << ","<< m.z  <<  "";
}

inline std::ostream & operator<<(std::ostream & out, const sb_float3 & m) {
    return out << "" <<  m.x << "," << m.y << ","<< m.z  <<  "";
}

inline std::ostream & operator<<(std::ostream & out, const sb_float4 & m) {
    return out << "" <<  m.x << "," << m.y << ","<< m.z << ","<< m.w <<  "" ;
}

template<typename type3 >
inline type3 atof3(const char * optarg) {
    type3 res;
    std::istringstream dotargs(optarg);
    std::string s;
    if (getline(dotargs, s, ',')) {
        res.x = atof(s.c_str());
    } else
        return res;
    if (getline(dotargs, s, ',')) {
        res.y = atof(s.c_str());
    } else {
        res.y = res.x;
        res.z = res.y;
        return res;
    }
    if (getline(dotargs, s, ',')) {
        res.z = atof(s.c_str());
    } else {
        res.z = res.y;
    }
    return res;
}
template<typename type3 >
inline type3 atoi3(const char * optarg) {
    type3 res;
    std::istringstream dotargs(optarg);
    std::string s;
    if (getline(dotargs, s, ',')) {
        res.x = atoi(s.c_str());
    } else
        return res;
    if (getline(dotargs, s, ',')) {
        res.y = atoi(s.c_str());
    } else {
        res.y = res.x;
        res.z = res.y;
        return res;
    }
    if (getline(dotargs, s, ',')) {
        res.z = atoi(s.c_str());
    } else {
        res.z = res.y;
    }
    return res;
}

template<typename type4 >
inline type4 atof4(const char * optarg) {
    type4 res;
    std::istringstream dotargs(optarg);
    std::string s;
    if (getline(dotargs, s, ',')) {
        res.x = atof(s.c_str());
    } else
        return res;
    if (getline(dotargs, s, ',')) {
        res.y = atof(s.c_str());
    } else {
        res.y = res.x;
        res.z = res.y;
        res.w = res.z;
        return res;
    }
    if (getline(dotargs, s, ',')) {
        res.z = atof(s.c_str());
    } else {
        res.z = res.y;
        res.w = res.z;
        return res;
    }
    if (getline(dotargs, s, ',')) {
        res.w = atof(s.c_str());
    } else {
        res.w = res.z;
    }
    return res;
}

struct Parameter {

    Parameter(std::string short_option, std::string name , std::string description, void (*callback)(Parameter*,ParameterComponent*) = nullptr) :
            short_option_(short_option), name_(name), description_(description), callback_(callback) {

    }
    virtual ~Parameter() {}
    inline  std::string getDescription() {return description_;}
    inline void (*getCallback())(Parameter*,ParameterComponent*) {return callback_;}

    inline  std::string           getName()                                                {return name_;}
    inline  const std::string     getShortOption(const ParameterComponent* pc)    const    {return (pc->getName() != "") ? pc->getName() + "-" + short_option_ : short_option_;}
    inline  std::string    		  getLongOption(const ParameterComponent* pc)     const    {return (pc->getName() != "") ? pc->getName() + "-" + name_ : name_;}


    virtual bool  requiresValue() = 0;
    virtual const  std::string   getStrValue() const = 0;
    virtual void          setValue(const char*) = 0;
    virtual void          resetValue()  = 0;
    virtual const std::string   getStrDefault()  = 0;
    virtual const std::string  getStrDetails(ParameterComponent* pc)  {
   	 std::stringstream res;
   	     res << "name:" << this->getLongOption(pc) << "\n" ;
   	     res << "short-option:" << this->getShortOption(pc)     << "\n" ;
         res << "description:" << this->getDescription()<< "\n" ;
         return res.str();
    }

protected :
    std::string    short_option_;
    std::string    name_;
    std::string    description_;
    void (*callback_)(Parameter*, ParameterComponent*);

};

struct TriggeredParameter : Parameter {

	TriggeredParameter(std::string shortOption,std::string name, std::string description, void (*callback)(Parameter*, ParameterComponent*) = nullptr) :
        Parameter(shortOption,name, description, callback) {
    }

    bool  requiresValue() {return false;};

    const  std::string   getStrValue() const {return triggered_ ? "true" : "false";};
    void                 setValue(const char*)  { triggered_ = true;};
    void                 resetValue() { triggered_ = false;};
    const std::string    getStrDefault()  {return "false";};


    virtual inline  const std::string   getStrDetails(ParameterComponent*pc)  {
   	 std::stringstream res;
       	 res << Parameter::getStrDetails(pc);
       	 res << "type:triggered\n" ;
         return res.str();
    }
private :
    bool triggered_ = false;
};

template<typename T>
struct TypedParameter : Parameter {

    TypedParameter(std::string shortOption,std::string name, std::string description, T* ptr,const T* default_v, void (*callback)(Parameter*, ParameterComponent*) = nullptr) :
            Parameter(shortOption,name, description, callback) ,
            default_value_(default_v) , ptr_(ptr) {
        	if (!ptr) {
    		throw std::logic_error("SLAMBench: Internal error, an instance of TypedParameter needs a valid storage pointer.");
    	}
    }

    const std::string getValue(const void*) const;
    void copyValue(T*,const T*);
    void setValue(const char* otarg);

    bool requiresValue() {return true;};


    inline  const std::string     getStrDefault()                       {if (default_value_ == nullptr) return "nullptr"; return getValue(default_value_);};
    inline  void                  resetValue()                          {if (ptr_ == nullptr) return; if (default_value_ == nullptr) return; copyValue((T*)ptr_, default_value_);};
    inline  const  std::string    getStrValue()                   const {if (ptr_ == nullptr) return "nullptr"; return getValue((T*)ptr_);};
    inline  const T&              getTypedValue()                       {if (ptr_ == nullptr) throw std::logic_error("Undefined Typed Parameter."); return *((T*)ptr_);};
    inline  const std::string     getStrType()                          {return  typeid(T).name();}


    virtual inline  const std::string   getStrDetails(ParameterComponent*pc)  {
   	 std::stringstream res;
       	 res << Parameter::getStrDetails(pc);
       	 res << "type:" << this->getStrType()<< "\n" ;
       	 res << "default:" << this->getStrDefault()<< "\n" ;
         return res.str();
    }

private :
    const T *default_value_;
    void *ptr_;

};

template<> inline const std::string  TypedParameter<float [2]>::getValue(const void * ptr) const {
	return "" + precise_to_string(((const float*)ptr)[0]) + "," + precise_to_string(((const float*)ptr)[1]) + "";
};

template<> inline void  TypedParameter<float [2]>::copyValue(float  (*to) [2] , float const (*from) [2]) {
	 (*to)[0] = (*from)[0];
	 (*to)[1] = (*from)[1];
};
template<> inline void  TypedParameter<float [2]>::setValue(const char* optarg)  {

	std::istringstream dotargs(optarg);
	std::string s;

	if (getline(dotargs, s, ',')) {
		((float*)ptr_)[0] = atof(s.c_str());
	}
	if (getline(dotargs, s, ',')) {
		((float*)ptr_)[1] = atof(s.c_str());
	}
};

template<> inline const std::string  TypedParameter<float [4]>::getValue(const void * ptr) const {
	return "" + precise_to_string(((const float*)ptr)[0]) + "," + precise_to_string(((const float*)ptr)[1]) + "," + precise_to_string(((const float*)ptr)[2]) + "," + precise_to_string(((const float*)ptr)[3]) + "";
};

template<> inline void  TypedParameter<float [4]>::copyValue(float  (*to) [4] , float const (*from) [4]) {
	 (*to)[0] = (*from)[0];
	 (*to)[1] = (*from)[1];
	 (*to)[2] = (*from)[2];
	 (*to)[3] = (*from)[3];
};

template<> inline void  TypedParameter<float [4]>::setValue(const char* optarg)  {

	std::istringstream dotargs(optarg);
	std::string s;

	if (getline(dotargs, s, ',')) {
		((float*)ptr_)[0] = atof(s.c_str());
	}
	if (getline(dotargs, s, ',')) {
			((float*)ptr_)[1] = atof(s.c_str());
	}
	if (getline(dotargs, s, ',')) {
			((float*)ptr_)[2] = atof(s.c_str());
	}
	if (getline(dotargs, s, ',')) {
			((float*)ptr_)[3] = atof(s.c_str());
	}
};

template<> inline void  TypedParameter<std::string>::copyValue(std::string* to,const std::string* from)    {*to = *(std::string*)from;;};
template<> inline void  TypedParameter<std::string>::setValue(const char*otarg)     {if (ptr_) (*(std::string*)ptr_) = std::string(otarg);};
template<> inline const std::string  TypedParameter<std::string>::getValue(const void * ptr) const {return *((std::string*) ptr);};

template<> inline void  TypedParameter<bool>::copyValue(bool* to,const bool* from)                {*to = *(bool*)from;;};
template<> inline void  TypedParameter<bool>::setValue(const char* otarg)                 {if (ptr_) (*(bool*)ptr_) = (std::string(otarg) == "true");};
template<> inline const std::string  TypedParameter<bool>::getValue(const void * ptr) const {return (*(bool*)ptr)?"true":"false";};


template<> inline void  TypedParameter<unsigned int>::copyValue(unsigned int* to,const unsigned int* from)              {*to = *(unsigned int*)from;;};
template<> inline void  TypedParameter<int>::copyValue(int* to,const int* from)              {*to = *(int*)from;;};
template<> inline void  TypedParameter<long unsigned int>::copyValue(long unsigned int* to,const long unsigned int* from)              {*to = *(long unsigned int*)from;;};
template<> inline void  TypedParameter<float>::copyValue(float* to,const float* from)            {*to = *(float*)from;;};
template<> inline void  TypedParameter<double>::copyValue(double* to,const double* from)           {*to = *(double*)from;;};
template<> inline void  TypedParameter<sb_float3>::copyValue(sb_float3* to,const sb_float3* from)        {*to = *(sb_float3*)from;;};
template<> inline void  TypedParameter<sb_float4>::copyValue(sb_float4* to,const sb_float4* from)        {*to = *(sb_float4*)from;;};
template<> inline void  TypedParameter<sb_uint3>::copyValue(sb_uint3* to,const sb_uint3* from)         {*to = *(sb_uint3*)from;;};
template<> inline void  TypedParameter< std::vector<int>  >::copyValue(std::vector<int> * to,const std::vector<int> * from)            {
    to->clear();
    for (int v : *from) {
        to->push_back(v);
    }

};
template<> inline void  TypedParameter< std::vector<std::string>  >::copyValue(std::vector<std::string> * to,const std::vector<std::string> * from)            {
    to->clear();
    for (auto v : *from) {
        to->push_back(v);
    }
};

template<> inline void  TypedParameter<unsigned int>::setValue(const char*otarg)             {if (ptr_) (*(unsigned int*)ptr_) = atoi(otarg);};
template<> inline void  TypedParameter<int>::setValue(const char*otarg)             {if (ptr_) (*(int*)ptr_) = atoi(otarg);};
template<> inline void  TypedParameter<long unsigned int>::setValue(const char*otarg)             {if (ptr_) (*(long unsigned int*)ptr_) = atoi(otarg);};
template<> inline void  TypedParameter<float>::setValue(const char*otarg)           {if (ptr_) (*(float*)ptr_) = atof(otarg);};
template<> inline void  TypedParameter<double>::setValue(const char*otarg)           {if (ptr_) (*(double*)ptr_) = atof(otarg);};
template<> inline void  TypedParameter<sb_float3>::setValue(const char*otarg)          {if (ptr_) (*(sb_float3*)ptr_)= atof3<sb_float3>(otarg);};
template<> inline void  TypedParameter<sb_float4>::setValue(const char*otarg)          {if (ptr_) (*(sb_float4*)ptr_)= atof4<sb_float4>(otarg);};
template<> inline void  TypedParameter<sb_uint3>::setValue(const char*otarg)           {if (ptr_) (*(sb_uint3*)ptr_) = atoi3<sb_uint3>(otarg);};
template<> inline void  TypedParameter< std::vector<int>  >::setValue(const char*from)           {
	if (ptr_ == nullptr) return;
	std::istringstream ss(from);
	std::string value;
	((std::vector<int>*)ptr_)->clear();
	while(std::getline(ss, value, ',')) {
		((std::vector<int>*)ptr_)->push_back(atoi(value.c_str()));
	}
};
template<> inline void  TypedParameter< std::vector<std::string>  >::setValue(const char*from)           {
	if (ptr_ == nullptr) return;
	std::istringstream ss(from);
	std::string value;
	((std::vector<std::string>*)ptr_)->clear();
	while(std::getline(ss, value, ',')) {
		((std::vector<std::string>*)ptr_)->push_back((value.c_str()));
	}
};

template<> inline const std::string  TypedParameter<unsigned int>::getValue(const void * ptr) const{return precise_to_string(*(unsigned int*)ptr);};
template<> inline const std::string  TypedParameter<int>::getValue(const void * ptr) const{return precise_to_string(*(int*)ptr);};
template<> inline const std::string  TypedParameter<long unsigned int>::getValue(const void * ptr) const{return precise_to_string(*(long unsigned int*)ptr);};
template<> inline const std::string  TypedParameter<float>::getValue(const void * ptr) const{return precise_to_string(*(float*)ptr);};
template<> inline const std::string  TypedParameter<double>::getValue(const void * ptr) const{return precise_to_string(*(double*)ptr);};
template<> inline const std::string  TypedParameter<sb_float3>::getValue(const void * ptr) const{
    return "" + precise_to_string(((sb_float3*)ptr)->x) + ","  +  precise_to_string(((sb_float3*)ptr)->y)  +  "," + precise_to_string(((sb_float3*)ptr)->z) + "";
};

template<> inline const std::string  TypedParameter<sb_float4>::getValue(const void * ptr) const{
    return "" +  precise_to_string(((sb_float4*)ptr)->x) + ","  +  precise_to_string(((sb_float4*)ptr)->y)  +  "," + precise_to_string(((sb_float4*)ptr)->z)+  "," + precise_to_string(((sb_float4*)ptr)->w) + "";
};

template<> inline const std::string  TypedParameter<sb_uint3>::getValue(const void * ptr) const{
    return "" + precise_to_string(((sb_uint3*)ptr)->x) + ","  +  precise_to_string(((sb_uint3*)ptr)->y)  +  "," + precise_to_string(((sb_uint3*)ptr)->z) + "";
};

template<> inline const std::string  TypedParameter< std::vector<int> >::getValue(const void * ptr) const{
    std::string res ;
    res = "" ;
    for (auto it = (*(std::vector<int>*)ptr).begin() ; it != (*(std::vector<int>*)ptr).end() ; it++ ) {
        if (it != (*(std::vector<int>*)ptr).begin() ) {
            res +=",";
        }
        res += precise_to_string(*it);

    }
    res +="";
    return res;
};
template<> inline const std::string  TypedParameter< std::vector<std::string> >::getValue(const void * ptr) const{
    std::string res = "" ;
    for (auto it = (*(std::vector<std::string>*)ptr).begin() ; it != (*(std::vector<std::string>*)ptr).end() ; it++ ) {
        if (it != (*(std::vector<std::string>*)ptr).begin() ) {
            res +=",";
        }
        res += *it;

    }
    res += "";
    return res;
};

template<typename T>
struct DiscretParameter : TypedParameter<T> {

	std::vector<T> values;

	DiscretParameter(std::vector<T> values , std::string shortOption,std::string name, std::string description, T* ptr,const T* default_v, void (*callback)(Parameter*, ParameterComponent*) = nullptr) :
            TypedParameter<T> (shortOption, name , description, ptr, default_v, callback) , values(values) {}

	virtual ~DiscretParameter() {}

    virtual inline  const std::string   getStrDetails(ParameterComponent* pc)  {
    	 std::stringstream res;
    	 res << TypedParameter<T>::getStrDetails(pc);
    	 for (auto value : this->values) {
           	 res << "values:" <<   this->getValue(&value) << "\n";
    	 }
      return res.str();
    }
};


template<typename T>
struct BoundedParameter : TypedParameter<T> {

	T min_, max_;

	BoundedParameter(T min, T max, std::string shortOption,std::string name, std::string description, T* ptr,const T* default_v, void (*callback)(Parameter*, ParameterComponent*) = nullptr) :
            TypedParameter<T> (shortOption, name , description, ptr, default_v, callback), min_(min), max_(max) {}

	virtual ~BoundedParameter() {}
    virtual inline  const std::string   getStrDetails(ParameterComponent* pc)  {
   	 std::stringstream res;
       	 res << TypedParameter<T>::getStrDetails(pc);
         res << "min:" << this->min_ << "\n";
         res << "max:" <<   this->max_ << "\n";
         return res.str();
    }
};




#endif /* FRAMEWORK_SHARED_INCLUDE_PARAMETERS_H_ */

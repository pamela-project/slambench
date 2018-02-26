/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef VALUEDISPATCH_H
#define VALUEDISPATCH_H

#include "Value.h"

namespace slambench {
	namespace values {
		class Value;
		
		class ValueDispatch {
		public:
			virtual ~ValueDispatch();
			
#define DISPATCH(vt) virtual void Dispatch(TypeForVT<vt>::type *value) = 0;
			DISPATCH(VT_COLLECTION);
			DISPATCH(VT_U64);
			DISPATCH(VT_DOUBLE);
			DISPATCH(VT_STRING);
			DISPATCH(VT_TRAJECTORY);
			DISPATCH(VT_POSE);
			DISPATCH(VT_POINTCLOUD);
			DISPATCH(VT_MATRIX);
#undef DISPATCH
		};
		
		class ConstValueDispatch {
		public:
			virtual ~ConstValueDispatch();
			
#define DISPATCH(vt) virtual void Dispatch(const TypeForVT<vt>::type *value) = 0;
			DISPATCH(VT_COLLECTION);
			DISPATCH(VT_U64);
			DISPATCH(VT_DOUBLE);
			DISPATCH(VT_STRING);
			DISPATCH(VT_TRAJECTORY);
			DISPATCH(VT_POSE);
			DISPATCH(VT_POINTCLOUD);
			DISPATCH(VT_MATRIX);
#undef DISPATCH
		};
	}
}

#endif /* VALUEDISPATCH_H */


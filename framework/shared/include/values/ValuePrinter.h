/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef VALUEPRINTER_H
#define VALUEPRINTER_H

#include "ValueDispatch.h"

#include <iostream>

namespace slambench {
	namespace values {
		
		class ValuePrinter : public ConstValueDispatch {
		public:
			ValuePrinter(std::ostream &str);
			virtual ~ValuePrinter();
			
			void Dispatch(const TypeForVT<VT_U64>::type* value) override;
			void Dispatch(const TypeForVT<VT_DOUBLE>::type* value) override;
			void Dispatch(const TypeForVT<VT_POINTCLOUD>::type* value) override;
			void Dispatch(const TypeForVT<VT_COLLECTION>::type* value) override;
			void Dispatch(const TypeForVT<VT_TRAJECTORY>::type* value) override;
			void Dispatch(const TypeForVT<VT_POSE>::type* value) override;
			void Dispatch(const TypeForVT<VT_MATRIX>::type* value) override;
			void Dispatch(const TypeForVT<VT_STRING>::type* value) override;

		private:
			std::ostream &str_;
		};
		
	}
}

#endif /* VALUEPRINTER_H */


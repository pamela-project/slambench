/*
 * IdentityFrame.h
 *
 *  Created on: Nov 27, 2018
 *      Author: toky
 */

#ifndef FRAMEWORK_SHARED_INCLUDE_IO_IDENTITYFRAME_H_
#define FRAMEWORK_SHARED_INCLUDE_IO_IDENTITYFRAME_H_

#include <io/SLAMFrame.h>

// IdentityFrame is a copy of a different frame, so it will access Data from the other frame.
class IdentityFrame : public slambench::io::SLAMFrame {
private:
	slambench::io::SLAMFrame * _from;
public :

	IdentityFrame(slambench::io::SLAMFrame * from) : _from (from) {
		this->FrameSensor = from->FrameSensor;
		this->Timestamp = from->Timestamp;
		if (this->FrameSensor->IsVariableSize()) {
			this->SetVariableSize(from->GetVariableSize());
		}

	}
	void *GetData() override { return _from->GetData();  };
	void FreeData() override { /* Data does not belong to this frame, no right to decide when to Free */};

};




#endif /* FRAMEWORK_SHARED_INCLUDE_IO_IDENTITYFRAME_H_ */

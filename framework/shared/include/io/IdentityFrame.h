/*
 * IdentityFrame.h
 *
 *  Created on: Nov 27, 2018
 *      Author: toky
 */

#ifndef FRAMEWORK_SHARED_INCLUDE_IO_IDENTITYFRAME_H_
#define FRAMEWORK_SHARED_INCLUDE_IO_IDENTITYFRAME_H_

#include <io/SLAMFrame.h>

class IdentityFrame : public slambench::io::SLAMFrame {
private:
	slambench::io::SLAMFrame * _from;
public :

	// TODO: I have some issues with this, we should have a way to copy SLAMFrame in a safer manner.
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

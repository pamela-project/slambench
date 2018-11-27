/*
 * identity.cpp
 *
 *  Created on: Nov 22, 2018
 *      Author: toky
 */

#include <SLAMBenchAPI.h>
#include <io/SLAMFrame.h>


class SLAMIdentityFrame : public slambench::io::SLAMFrame {
private:
	slambench::io::SLAMFrame * _from;
public :

	// TODO: I have some issues with this, we should have a way to copy SLAMFrame in a safer manner.
	SLAMIdentityFrame(slambench::io::SLAMFrame * from) : _from (from) {
		this->FrameSensor = from->FrameSensor;
		this->Timestamp = from->Timestamp;
		if (this->FrameSensor->IsVariableSize()) {
			this->SetVariableSize(from->GetVariableSize());
		}

	}
	void *GetData() override { return _from->GetData();  };
	void FreeData() override { return _from->FreeData(); };

};



 slambench::io::SLAMFrame * sb_filter (SLAMBenchFilterLibraryHelper * , slambench::io::SLAMFrame * frame) {
	return new SLAMIdentityFrame(frame);
}



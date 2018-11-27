/*
 * skip_frame.cpp
 *
 *  Created on: Nov 22, 2018
 *      Author: toky
 */

#include <SLAMBenchAPI.h>

 slambench::io::SLAMFrame * sb_filter (SLAMBenchFilterLibraryHelper * , slambench::io::SLAMFrame * frame) {

	 // TODO: need to skip same sensor frames, not across sensors

	static int coin = 0;
	coin = !coin;
	if (coin) {
		return frame;
	} else {
		return nullptr; // we skip so return false to say not ready.
	}
}







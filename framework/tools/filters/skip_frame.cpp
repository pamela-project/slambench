/*
 * skip_frame.cpp
 *
 *  Created on: Nov 22, 2018
 *      Author: toky
 */

#include <SLAMBenchAPI.h>
#include <io/IdentityFrame.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/CameraSensorFinder.h>

double threshold = 0;
double default_threshold = 0.2;
static slambench::io::DepthSensor *depth_sensor;
std::map<slambench::io::Sensor*,slambench::io::SLAMFrame *> last_frames;

bool compare_depth_images (short * previous , short * current) {
	assert(previous != current);
	return false;
}
bool compare (slambench::io::SLAMFrame * previous , slambench::io::SLAMFrame * current) {
	double result = 0.0;

	assert(previous->FrameSensor == current->FrameSensor);

	if (current->FrameSensor == depth_sensor) {
		std::cout << "Test the frame..." << std::endl;
		return compare_depth_images((short *) previous->GetData(), (short *) current->GetData());
	} else {
		std::cout << "Wrong sensor" << std::endl;
		return false;
	}


}


bool sb_new_filter_configuration (SLAMBenchFilterLibraryHelper * filter_settings) {
	std::cout << "** sb_new_filter_configuration Init depth sensor." << std::endl;
	filter_settings->addParameter(TypedParameter<double>("fth", "skip-threshold",     "Numerical value to specify when the threshold is reached.",    &threshold, &default_threshold));
	return true;
}

bool sb_init_filter (SLAMBenchFilterLibraryHelper * filter_settings) {

	std::cout << "** sb_init_filter Init depth sensor." << std::endl;

	slambench::io::CameraSensorFinder sensor_finder;
	depth_sensor = (slambench::io::DepthSensor*)sensor_finder.FindOne(filter_settings->get_sensors(), {{"camera_type", "depth"}});
	assert(depth_sensor);
	return true;
}

 slambench::io::SLAMFrame * sb_process_filter (SLAMBenchFilterLibraryHelper * , slambench::io::SLAMFrame * frame) {

	 slambench::io::SLAMFrame * new_frame = nullptr;
	 if (last_frames.find(frame->FrameSensor) != last_frames.end()) {

		 // previous frame found.
		 if (compare (frame, last_frames.at(frame->FrameSensor))) {
			 std::cout << "Skip one frame." << std::endl;
		 } else {
			 new_frame = new IdentityFrame(frame);
		 }

	 } else {
		 std::cout << "(last_frames.find(frame->FrameSensor) return end" << std::endl;
		 new_frame = new IdentityFrame(frame);
	 }

	 last_frames.insert(std::pair<slambench::io::Sensor*,slambench::io::SLAMFrame *>(frame->FrameSensor,frame));

	 return new_frame;

}







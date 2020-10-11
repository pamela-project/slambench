/*

 Copyright (c) 2020 University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */
#include <iostream>
#include <io/FrameBufferSource.h>
#include <io/openni15/ONI15InputInterface.h>
#include <io/openni2/ONI2InputInterface.h>
#include <io/realsense/RealSense2InputInterface.h>
#include <io/InputInterfaceManager.h>

using namespace slambench::io;

InputInterfaceManager::InputInterfaceManager(const std::vector<std::string> & dataset_filenames) {
#ifdef DO_OPENNI20
    if (dataset_filenames[0] == "oni2") {
        std::cerr << "Load OpenNI 2 interface ..." << std::endl;
        input_interfaces_.push_back(new slambench::io::openni2::ONI2InputInterface());
        return;
    }
#endif
#ifdef DO_OPENNI15
    if (dataset_filenames[0] == "oni15") {
            std::cerr << "Load OpenNI 1.5 interface ..." << std::endl;
            input_interfaces_.push_back(new slambench::io::openni15::ONI15InputInterface());
            return;
        }
#endif
#ifdef DO_REALSENSE
    if (dataset_filenames[0] == "realsense") {
        std::cerr << "Load RealSense interface ..." << std::endl;
        auto interface = new slambench::io::realsense::RealSense2InputInterface();
        input_interfaces_.push_back(interface);
        input_stream_ = &interface->GetStream();
        return;
    }
#endif
    // TODO: Handle other types of interface
    // TODO: Add a getFrameStream in Config to handle that
    // TODO: config will be aware of sensors and then sensors will be able to add there arguments
    for(const auto &filename : dataset_filenames) {

        FILE *input_desc = fopen(filename.c_str(), "r");
        if (input_desc == nullptr) {
            throw std::logic_error("Could not open the input file");
        }
        auto input_ref = new slambench::io::FileStreamInputInterface(input_desc,
                                                                     new slambench::io::SingleFrameBufferSource());
        input_interfaces_.push_back(input_ref);
        ////workaround to be compatible with benchmarks that does not implement sensors resetting.
        ////assume different input_interface_manager_ has exactly the same types of sensors.
        ////If sensors are different, may introduce problems.
        // FIXME: this works for a single library but doesn't work for more than that. Should be moved inside each library
    }
    first_sensors_ = &GetCurrentInputInterface()->GetSensors();
}

InputInterfaceManager::~InputInterfaceManager() {
    for(auto interface : input_interfaces_)
        delete interface;
    if(first_sensors_)
        delete first_sensors_;
    if(input_stream_)
        delete input_stream_;
}

InputInterface* InputInterfaceManager::GetCurrentInputInterface()
{
    if(input_interfaces_.empty()) {
         throw std::logic_error("Input interface has not been added to SLAM configuration");
    }
    return input_interfaces_.front();
}

SLAMFrame* InputInterfaceManager::GetNextFrame() const {
    if (input_stream_ == nullptr || !input_stream_->HasNextFrame()) {
        std::cerr << "No input loaded." << std::endl;
        return nullptr;
    }

    return input_stream_->GetNextFrame();
}

SLAMFrame* InputInterfaceManager::GetClosestGTFrameToTime(slambench::TimeStamp& ts) const {
    return dynamic_cast<slambench::io::GTBufferingFrameStream*>(input_stream_)->GetGTFrames()->GetClosestFrameToTime(ts);
}

bool InputInterfaceManager::LoadNextInputInterface()
{
    input_interfaces_.pop_front();
    delete input_stream_;
    if(input_interfaces_.empty())
        return false;
    updated_ = true;

    GetCurrentInputInterface()->GetSensors() = *first_sensors_;
    return true;
}

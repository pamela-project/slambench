/*

 Copyright (c) 2020 University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef IO_INPUTINTERFACEMANAGER_H
#define IO_INPUTINTERFACEMANAGER_H

#include <cstring>

#include <io/FrameSource.h>
#include <io/SLAMFile.h>

#include "sensor/SensorCollection.h"
#include "deserialisation/SLAMFrameDeserialiser.h"
#include "InputInterface.h"
#include <list>
namespace slambench {
    namespace io {

        class InputInterfaceManager {
        public:
            InputInterfaceManager(const std::vector<std::string>& dataset_filenames);
            ~InputInterfaceManager();

            slambench::io::InputInterface *GetCurrentInputInterface();

            SLAMFrame* GetNextFrame() const;
            SLAMFrame* GetClosestGTFrameToTime(slambench::TimeStamp& ts) const;
            bool LoadNextInputInterface();
            bool initialized()
            {
                return input_interfaces_.empty();
            }
            slambench::io::FrameStream *input_stream_;
            bool updated_ = false;
        private:
            std::list<slambench::io::InputInterface *> input_interfaces_;
            slambench::io::SensorCollection* first_sensors_;

        };
    }
}

#endif // IO_INPUTINTERFACEMANAGER_H

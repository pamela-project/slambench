/*

 Copyright (c) 2019 Intel Corp.

 This code is licensed under the MIT License.

 */


#ifndef RESULT_WRITER_H_
#define RESULT_WRITER_H_

#include <outputs/Output.h>
#include <fstream>

class ResultWriter {
public:
    ResultWriter(std::ofstream &stream);

    void WriteKV(std::string key, std::string value);

    void WriteKV(std::string key, std::vector<std::string> values, std::string separator=",");

    void WriteTrajectory(slambench::outputs::BaseOutput::value_map_t traj);

    static std::string GetCPUModel();

    static std::string GetMemorySize();

private:
    std::ofstream &out_;
};

#endif

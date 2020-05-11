/*

 Copyright (c) 2019 Intel Corp.

 This code is licensed under the MIT License.

 */

#include "ResultWriter.h"
#include <iostream>

ResultWriter::ResultWriter(std::ofstream &out_stream):out_(out_stream) {}

void ResultWriter::WriteKV(std::string key, std::string value)
{
    out_ << key << ": " << value << std::endl;
}

void ResultWriter::WriteKV(std::string key, std::vector<std::string> values, std::string separator)
{
    out_ << key << ": ";
    for (auto it = values.begin(); it != values.end(); ++it) {
        if (it != values.begin()) out_ << separator;
        out_ << *it;
    }
    out_ << std::endl;
}

void ResultWriter::WriteTrajectory(slambench::outputs::BaseOutput::value_map_t traj)
{
	float x, y, z;
	Eigen::Matrix3d R;
    out_ << "# data_stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w" << std::endl;
    for (auto point: traj) {
        Eigen::Matrix4f pose = dynamic_cast<const slambench::values::PoseValue*>(point.second)->GetValue();
        x = pose(0, 3);
        if (std::to_string(x) == "-nan") {
            continue;
        }
        y = pose(1, 3);
        z = pose(2, 3);
        Eigen::Matrix3f R = pose.block<3,3>(0,0);
        Eigen::Quaternionf q(R);
        q.normalize();
        out_ << point.first << " " << x << " " << y << " " << z << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() <<std::endl;
	}
}

std::string ResultWriter::GetCPUModel()
{
    std::string cpu_info;
    std::string::size_type position;
    FILE *fp = fopen("/proc/cpuinfo", "r");
    if(NULL == fp)
        std::cerr << "failed to open cpuinfo\n";
    char szTest[1000] = {0};
    while(!feof(fp))
    {
        memset(szTest, 0, sizeof(szTest));
        fgets(szTest, sizeof(szTest) - 1, fp);
        cpu_info = szTest;
        position = cpu_info.find("model name");
        if (position != cpu_info.npos && position == 0) {
            position = cpu_info.find(":");
            cpu_info = cpu_info.substr(position + 1, 100);
            position = cpu_info.find("\n");
            if (position != cpu_info.npos) {
                cpu_info = cpu_info.substr(0, position);
            }
            break;
        }
    }
    fclose(fp);
    return cpu_info;
}

std::string ResultWriter::GetMemorySize()
{
    int mem_size = 0;
    std::string mem_info;
    std::string::size_type position;
    FILE *fp = fopen("/proc/meminfo", "r");
    if(NULL == fp)
        std::cerr << "failed to open meminfo\n";
    char szTest[1000] = {0};
    while(!feof(fp))
    {
        memset(szTest, 0, sizeof(szTest));
        fgets(szTest, sizeof(szTest) - 1, fp);
        mem_info = szTest;
        position = mem_info.find("MemTotal");
        if (position != mem_info.npos && position == 0) {
            position = mem_info.find(":");
            mem_info = mem_info.substr(position + 1, 100);
            position = 0;
            for(size_t i = 0; i < mem_info.size(); i++) {
                if (int(mem_info[i]) >= 48 && int(mem_info[i]) <= 57) {
                    break;
                }
                position++;
            }
            mem_info = mem_info.substr(position, 100);
            position = 0;
            for(size_t i = 0; i < mem_info.size(); i++) {
                if (int(mem_info[i]) < 48 || int(mem_info[i]) > 57) {
                    break;
                }
                position++;
            }
            mem_info = mem_info.substr(0, position);
            mem_size = std::stoi(mem_info);
            break;
        }
    }
    fclose(fp);
    return std::to_string(mem_size / 1024 / 1024) + " GB";
}

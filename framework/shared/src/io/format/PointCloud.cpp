/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/format/PointCloud.h"

#include <boost/regex.hpp>
#include <string>

using namespace slambench::io;

PointCloud* PointCloud::FromRaw(char* data)
{
	uint32_t count = *(uint32_t*)data;
	
	float *fdata = (float*)(data+4);
	
	PointCloud *pc = new PointCloud();
	
	for(uint32_t i = 0; i < count; ++i) {
		float x = fdata[i*3];
		float y = fdata[i*3+1];
		float z = fdata[i*3+2];
		pc->Get().push_back(Point(x,y,z));
	}
	
	return pc;
}

std::vector<char> PointCloud::ToRaw()
{
	std::vector<char> output(4 + (Get().size() * 12));
	*(uint32_t*)(output.data()) = Get().size();
	
	float *fdata = (float*)(output.data()+4);
	for(uint32_t i = 0; i < Get().size(); ++i) {
		fdata[i*3] = Get().at(i).x;
		fdata[i*3+1] = Get().at(i).y;
		fdata[i*3+2] = Get().at(i).z;
	}
	
	return output;
}


// very basic PLY file format reader
PointCloud *PlyReader::Read(std::istream &input)
{
	// read the PLY header
	int point_size = 0;
	int point_y_offset = -1;
	int point_z_offset = -1;
	int point_x_offset = -1;
	int point_count = -1;
	
	boost::regex rgx ("^([^ ]+)\\s*([^ ]+)?\\s*([^ ]+)?\\s*$");
	
	do {
		std::string line;
		std::getline(input, line);
		
		// figure out the type of line
		boost::cmatch cm;
		boost::regex_search (line.c_str(), cm, rgx);
		
		if(cm.size() == 0) {
			printf("Line does not match expected format\n");
			printf("%s\n", line.c_str());
			return nullptr;
		}
		
		const std::string &type = cm[1];
		
		if(type == "ply") {
			printf("Found ply header\n");
			// so far so good
		} else if(type == "format") {
			if(cm[2] != "binary_little_endian") {
				printf("Invalid format type '%s'\n", cm[2].str().c_str());
				return nullptr;
			}
			if(cm[3] != "1.0") {
				printf("Invalid version\n");
				return nullptr;
			}
		} else if(type == "element") {
			if(cm[2] != "vertex") {
				printf("Invalid element type\n");
				return nullptr;
			}
			
			point_count = strtol(cm[3].str().c_str(), nullptr, 10);
			printf("Located point count\n");
			
		} else if(type == "property") {
			const std::string &ptype = cm[2];
			const std::string &pname = cm[3];
			
			if(ptype == "float") {
				if(pname == "x") {
					point_x_offset = point_size;
					printf("Located point x\n");
				} else if(pname == "y") {
					point_y_offset = point_size;
					printf("Located point y\n");
				} else if(pname == "z") {
					point_z_offset = point_size;
					printf("Located point z\n");
				}
				point_size += 4;
			} else if(ptype == "uchar") {
				point_size += 1;
			} else {
				return nullptr;
			}
		} else if(type == "end_header") {
			break;
		} else {
			printf("Unexpected line type '%s'\n", cm[1].str().c_str());
			return nullptr;
		}
		
	} while(input.good());
	
	// make sure we have read all of the important values
	if(point_count == -1 || point_size == 0 || point_x_offset == -1 || point_y_offset == -1 || point_z_offset == -1) {
		printf("Did not locate expected element types\n");
		return nullptr;
	}
	
	// start parsing points
	PointCloud *pc = new PointCloud();
	std::vector<char> pointdata (point_count);
	for(int i = 0; i < point_count; ++i) {
		char *pd = pointdata.data();
		// read data
		input.read(pd, point_size);
		
		Point p;
		p.x = *(float*)(pd + point_x_offset);
		p.y = *(float*)(pd + point_y_offset);
		p.z = *(float*)(pd + point_z_offset);
		
		pc->Get().push_back(p);
	}
	
	return pc;
}

void PlyWriter::Write(const PointCloud* cloud, std::ostream& file)
{
	file << "ply" << std::endl;
	file << "format binary_little_endian 1.0" << std::endl;
	file << "element vertex " << cloud->Get().size() << std::endl;
	file << "property float x" << std::endl;
	file << "property float y" << std::endl;
	file << "property float z" << std::endl;
	file << "end_header" << std::endl;
	for(auto point : cloud->Get()) {
		struct { float x, y, z; } data;
		data.x = point.x;
		data.y = point.y;
		data.z = point.z;
		file.write((char*)&data, sizeof(data));
	}
}

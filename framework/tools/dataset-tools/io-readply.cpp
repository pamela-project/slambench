/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "io/format/PointCloud.h"

#include <fstream>

int main(int argc, char **argv) {
	slambench::io::PlyReader reader;
	std::ifstream str (argv[1]);
	auto pc = reader.Read(str);
	
	if(pc == nullptr) {
		printf("Could not read pointcloud!\n");
		return 1;
	} else {
		for(auto p : pc->Get()) {
			printf("%f %f %f\n", p.x, p.y, p.z);
		}
	}
	return 0;
}

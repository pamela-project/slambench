/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef IO_POINTCLOUD_H
#define IO_POINTCLOUD_H

#include <iostream>
#include <vector>

namespace slambench {
	namespace io {
		class Point {
		public:
			Point() : x(0), y(0), z(0) {}
			Point(float x, float y, float z) : x(x), y(y), z(z) {}
			
			float x, y, z;
		};
		
		class PointCloud {
		public:
			typedef std::vector<Point> storage_t;
			
			storage_t &Get() { return points_; }
			const storage_t &Get() const { return points_; }
			
			static PointCloud *FromRaw(char*);
			std::vector<char> ToRaw();
			
		private:
			storage_t points_;
		};
		
		class PlyReader {
		public:
			PointCloud *Read(std::istream &file);
		};
		
		class PlyWriter {
		public:
			void Write(const PointCloud *cloud, std::ostream &file);
		};
	}
}

#endif /* IO_POINTCLOUD_H */


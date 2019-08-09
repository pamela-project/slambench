/*

 Copyright (c) 2019 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_UTILS_PLYASCIIREADER_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_UTILS_PLYASCIIREADER_H_

#include <fstream>
#include <string>
#include <memory>
#include <vector>
#include <algorithm>

#include <io/format/PointCloud.h>

#include "timings.h"

// Due to -Werror flag the implementation of tinyply fails, these directive allow it to build
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wswitch"
#pragma GCC diagnostic ignored "-Wsign-compare"
#define TINYPLY_IMPLEMENTATION
#include "./tinyply.h"
#pragma GCC diagnostic pop


class PlyASCIIReader {
 public:
  static slambench::io::PointCloud* read(const std::string &filepath) {

    using namespace tinyply;
    using namespace slambench::io;

    std::ifstream ss(filepath, std::ios::binary);
    if (ss.fail()) {
      throw std::runtime_error("failed to open " + filepath);
    }

    PlyFile file;
    file.parse_header(ss);

    std::cout << "Reading ASCII Plyfile: " << filepath << std::endl;
    std::cout << "........................................................................\n";
    for (const auto& c : file.get_comments()) std::cout << "Comment: " << c << std::endl;
    for (const auto& e : file.get_elements()) {
      std::cout << "element - " << e.name << " (" << e.size << ")" << std::endl;
      for (const auto& p : e.properties) {
        std::cout << "\tproperty - " << p.name
                  << " (" << tinyply::PropertyTable[p.propertyType].str << ")" << std::endl;
      }
    }
    std::cout << "........................................................................\n";

    // tinyply treats parsed data as untyped byte buffers.
    std::shared_ptr<PlyData> points_data;
    const int list_size_hint = 54000000;

    try {
      points_data = file.request_properties_from_element("vertex",
                                                         {"x", "y", "z" },
                                                         list_size_hint);
    } catch (const std::exception & e) {
      std::cerr << "tinyply exception: " << e.what() << std::endl;
    }

    double start = tock();
    file.read(ss);
    double end = tock();

    std::cout << "Reading took " << (end-start) << " seconds." << std::endl;
    std::cout << "Read " << points_data->count << " total points "<< std::endl;

    // move from untyped bytes buffers to point objects
    const size_t numBytes = points_data->buffer.size_bytes();
    std::vector<Point> points(points_data->count);

    std::copy(points_data->buffer.get(),
              points_data->buffer.get() + numBytes,
              reinterpret_cast<uint8_t*>(points.data()));

    // create pointcloud and assign points
    auto pc = new PointCloud();
    pc->Get() = points;

    return pc;
  }
};

#endif  // FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_UTILS_PLYASCIIREADER_H_

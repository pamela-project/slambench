/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of
 Manchester. Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include <ParameterComponent.h>
#include <ParameterManager.h>
#include <Parameters.h>

#include <io/SLAMFile.h>
#include <io/SLAMFrame.h>
#include <io/format/PointCloud.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/PointCloudSensor.h>
#include <io/sensor/Sensor.h>
#include <Eigen/Eigen>

#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>

#include <cmath>

#include <unistd.h>

#include "./include/BONN.h"
#include "./include/EUROCMAV.h"
#include "./include/ICL.h"
#include "./include/ICLNUIM.h"
#include "./include/SVO.h"
#include "./include/TUM.h"

using namespace slambench::io;

class MainComponent : public ParameterComponent {
 private:
  slambench::ParameterManager param_manager;
  DatasetReader* reader = nullptr;

 public:
  std::string binary_name;
  std::string dataset;
  std::string output;
  bool quiet;

 public:
  static void help_callback(Parameter*, ParameterComponent* caller) {
    MainComponent* config = dynamic_cast<MainComponent*>(caller);

    std::cerr << "" << std::endl;
    std::cerr << " == =========================== ==" << std::endl;
    std::cerr << " == SLAMBench Dataset Generator ==" << std::endl;
    std::cerr << " == =========================== ==" << std::endl;
    std::cerr << "" << std::endl;
    std::cerr << "Description: "
              << "The dataset generator is used to produce *.slam files "
                 "(dataset files compatible with the SLAMBench loader)."
              << "" << std::endl;

    std::cerr << "Usage: " << config->binary_name
              << " -d dataset-type -o output-slam-file [ with extra arguments "
                 "required for the different dataset type]"
              << std::endl;
    std::cerr << "  Available parameters :" << std::endl;
    config->param_manager.PrintArguments(std::cerr);

    exit(0);
  }

  static void dataset_callback(Parameter* p, ParameterComponent* caller) {
    MainComponent* config = dynamic_cast<MainComponent*>(caller);
    TypedParameter<std::string>* dataset = dynamic_cast<TypedParameter<std::string>*>(p);
    std::string dataset_name = dataset->getTypedValue();

    if (dataset_name == "iclnuim") {
      config->reader = new ICLNUIMReader("");
    } else if (dataset_name == "tum") {
      config->reader = new TUMReader("");
    } else if (dataset_name == "eurocmav") {
      config->reader = new EUROCMAVReader("");
    } else if (dataset_name == "icl") {
      config->reader = new ICLReader("");
    } else if (dataset_name == "svo") {
      config->reader = new SVOReader("");
    } else if (dataset_name == "bonn") {
      config->reader = new BONNReader("");
    }

    if (config->reader) {
      config->param_manager.AddComponent(config->reader);
    } else {
      std::cout << "Data format not found" << std::endl;
      exit(1);
    }
  }

  static void frame_callback(int idx, int total) {
    printf("\r");

    // print progress bar
    printf("[");
    const int width = 50;
    float blocks = width * ((float)idx / total);

    int i = 0;
    for (; i < blocks; ++i) {
      printf("#");
    }
    for (; i < width; ++i) {
      printf(" ");
    }
    printf("] ");

    printf("%u / %u", idx, total);
    fflush(stdout);
  }
  MainComponent(int argc, char* argv[]) : ParameterComponent(""), binary_name(argv[0]) {

      this->addParameter(TypedParameter<std::string>("d", "dataset",
                                                     "Name of the input dataset type "
                                                     "(iclnuim, tum, eurocmav, icl, svo, bonn)",
                                                     &this->dataset, NULL, this->dataset_callback));

      this->addParameter(TypedParameter<std::string>("o", "log-file",
                                                     "Output slam file",
                                                     &this->output, NULL));

      this->addParameter(TypedParameter<bool>("q", "quiet",
                                              "Hide the progress bar",
                                              &this->quiet, NULL));

      this->addParameter(TriggeredParameter("h", "help",
                                            "Print the help.",
                                            this->help_callback));

    this->param_manager.AddComponent(this);
    this->param_manager.ReadArguments(argc, argv, this);
  }

  bool Run() {
    SLAMFile* slamfile = reader->GenerateSLAMFile();
    if (slamfile == nullptr) {
      std::cout << "No SLAM File generated." << std::endl;
      return false;
    }
    std::cout << "Writing output." << std::endl;

    return SLAMFile::Write(this->output, *slamfile,
                           this->quiet ? nullptr : frame_callback);
  }
};

int main(int argc, char* argv[]) {
  MainComponent* main = new MainComponent(argc, argv);

  if (main->dataset == "") {
      std::cout << " Please define the dataset type using the -d argument.\n"
                   " Possible values are: iclnuim tum eurocmav icl svo bonn\n"
                   " To have details of arguments for any type of dataset you are interested by,\n"
                   " Please run the help mode for this dataset (e.g " << argv[0] << "-d tum)\n";
    return EXIT_FAILURE;
  }

  if (main->output == "") {
    std::cout << " Please define the output file using the -o argument. " << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "Selected Dataset is " << main->dataset << std::endl;
  std::cout << "Selected output is " << main->output << std::endl;

  std::cout << "Run generation..." << std::endl;

  bool res = main->Run();

  std::cout << std::endl << "Done." << std::endl;

  assert(res);
}

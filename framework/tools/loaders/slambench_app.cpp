/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include <SLAMBenchConfiguration.h>
#include <outputs/OutputManagerWriter.h>
#include <ColumnWriter.h>
#include <SLAMBenchException.h>
#include <SLAMBenchAPI.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

std::string default_output_filename;
std::string output_filename;
bool use_gui, default_use_gui = false;
bool lifelong_slam, default_lifelong_slam = false;

std::string alignment_technique = "new";
std::string default_alignment_technique = "new";
TypedParameter<std::string> file_output_parameter ("fo", "file-output", "File to write slamfile containing outputs", &output_filename, &default_output_filename);
TypedParameter<std::string> alignment_type_parameter("a",     "alignment-technique",      "Select an alignment technique by name, if not found, \"new alignment\" used (original,new,umeyama).", &alignment_technique, &default_alignment_technique);
TypedParameter<bool> gui_parameter("gui", "gui", "Whether or not to display the graphical user interface", &use_gui, &default_use_gui);
TypedParameter<bool> lifelong_parameter("ll",     "lifelong",      "If given multiple sequences, relocalise in between sequences rather than starting a new ", &lifelong_slam, &default_lifelong_slam);

#ifdef WITH_GUI
#include <SLAMBenchUI_Pangolin.h>
#include <pangolin/pangolin.h>
static SLAMBenchUI* volatile ui = nullptr;
void run_pangolin(bool *stay_on, SLAMBenchConfiguration *config) {
    std::cerr << "Creation of GUI interface." << std::endl;
    auto ui_pangolin = new SLAMBenchUI_Pangolin();
    ui_pangolin->AddOutputManager("Ground Truth", &config->GetGroundTruth());
    for (auto lib : config->GetLoadedLibs()) {
        ui_pangolin->AddOutputManager(lib->getName(), &lib->GetOutputManager());
    }
    ui_pangolin->InitialiseOutputs();
    ui = dynamic_cast<SLAMBenchUI *> (ui_pangolin);

    // Provide the current camera position to the GUI
    ui->update_camera(480, 640, 520, 521, 324, 249);

    std::cerr << "Start rendering loop." << std::endl;
    while (!pangolin::ShouldQuit()) {

        usleep(40000);
        if (!ui->process()) {
            std::cerr << "Rendering problem." << std::endl;
            exit(1);
        }
    }

    //***************************************************************************************
    // FINISH RENDERING
    //***************************************************************************************
    std::cout << "Stop Pangolin..." << std::endl;
    pangolin::Quit();
    std::cout << "GUI closed ... " << std::endl;


    //***************************************************************************************
    // CLOSE COMPUTE THREADS
    //***************************************************************************************
    *stay_on = false;
    std::cout << "Wait for compute thread..." << std::endl;
    //	compute_thread.join();
}
#endif

int main(int argc, char * argv[])
{
    try {
        auto config = new SLAMBenchConfiguration();

        config->addParameter(file_output_parameter);
        config->addParameter(alignment_type_parameter);
        config->addParameter(gui_parameter);
        config->addParameter(lifelong_parameter);

        // At this point the datasets/libraries/sensors are loaded with their arguments set.
        config->GetParameterManager().ReadArgumentsOrQuit(argc, argv);

        // Initialise the configuration, retrieve the ground truth and set the alignement
        config->InitGroundtruth();

        // Prepare the logging
        config->StartStatistics();

        //***************************************************************************************
        // We init the algos now because we need their output already
        // TODO: if pose and map were by default we could init the algo much later,
        //       thus move memory metric later
        //***************************************************************************************
        config->InitAlgorithms();
        config->InitAlignment();
        config->InitWriter();
        // Run the experiment
        if(use_gui)
        {
#ifdef WITH_GUI
            std::thread pangolin_thread(run_pangolin, &use_gui, config);
            while(ui == nullptr) ; // spin until UI is initialised
            config->ComputeLoopAlgorithm(&use_gui, ui);
            pangolin_thread.join();
#else
            std::cerr<< "Not compiled with Pangolin support! Continuing without GUI." <<std::endl;
            SLAMBenchConfiguration::ComputeLoopAlgorithm(config, nullptr, nullptr);
#endif
        } else {
            config->ComputeLoopAlgorithm(nullptr, nullptr);
        }

        // End of experiment, we output the map
        // TODO: Only one output file does not do the job for more than one SLAM systems, output directory maybe ?
        if(!output_filename.empty()) {
            if(config->GetLoadedLibs().size() > 1) {
                std::cerr << "Can only write outputs to file when there is only one lib loaded" << std::endl;
                return 1;
            }
            // enable all writeable outputs
            auto main_lib = config->GetLoadedLibs().front();

            main_lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE)->SetActive(true);
            main_lib->GetOutputManager().GetMainOutput(slambench::values::VT_POINTCLOUD)->SetActive(true);
            main_lib->GetOutputManager().GetMainOutput(slambench::values::VT_FRAME)->SetActive(true);
            auto timestamp = main_lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE)->GetMostRecentValue().first;

            main_lib->c_sb_update_outputs(main_lib, &timestamp);

            std::cout << "Writing outputs to " << output_filename << std::endl;
            slambench::outputs::OutputManagerWriter omw;
            omw.Write(main_lib->GetOutputManager(), output_filename);
            std::cout << "Done writing outputs." << std::endl;
        }

        std::cout << "End of program." << std::endl;
        delete config;

    } catch (const SLAMBenchException& e) {

        std::cout << "An error occurred during the execution." << std::endl;
        std::cout << e.what() << std::endl;

    }
    return 0;
}

/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include <SLAMBenchAPI.h>
#include <SLAMBenchConfiguration.h>
#include <SLAMBenchUI_Pangolin.h>
#include <thread>

#include <io/FrameBufferSource.h>
#include <io/SLAMFrame.h>
#include <SLAMBenchException.h>
#include <unistd.h>
#include <cstdlib>


std::string alignment_technique = "new";
std::string default_alignment_technique = "new";
TypedParameter<std::string> alignment_type_parameter("a",     "alignment-technique",      "Select an alignment technique by name, if not found, \"new alignment\" used (original,new,umeyama).", &alignment_technique, &default_alignment_technique);

void run_pangolin(bool *stay_on, SLAMBenchConfiguration *config);
static SLAMBenchUI * volatile ui = nullptr;

int main(int argc, char * argv[])
{
    try {
        auto config = new SLAMBenchConfiguration();

        //***************************************************************************************
        // Start the argument processing
        //***************************************************************************************
        config->getParameters().push_back(&alignment_type_parameter);
        config->GetParameterManager().ReadArgumentsOrQuit(argc, argv, config);
        //***************************************************************************************
        // At this point the datasets/libraries/sensors are loaded with their arguments set.
        //***************************************************************************************


        //***************************************************************************************
        // Initialise the configuration, i.e retrieve ground truth and set the alignment
        //***************************************************************************************
        config->InitGroundtruth();

        // get GT trajectory
        auto gt_trajectory = config->GetGroundTruth().GetMainOutput(slambench::values::VT_POSE);

        slambench::outputs::TrajectoryAlignmentMethod *alignment_method;
        if(alignment_technique == "original") {
            alignment_method = new slambench::outputs::OriginalTrajectoryAlignmentMethod();
        } else if(alignment_technique == "new") {
            alignment_method = new slambench::outputs::NewTrajectoryAlignmentMethod();
        }  else if(alignment_technique == "umeyama") {
            alignment_method = new slambench::outputs::UmeyamaTrajectoryAlignmentMethod();
        } else {
            std::cerr << "Unknown alignment method " << alignment_technique << std::endl;
            return 1;
        }

        //***************************************************************************************
        // We prepare the logging and create the global metrics
        //***************************************************************************************

        //***************************************************************************************
        // Init the algos now because we need their output already
        // TODO: if pose and map were by default we could init the algo much later,
        //       thus move memory metric later
        //***************************************************************************************
        config->InitAlgorithms();

        // If a ground truth is available, produce an aligned trajectory for each algorithm
        slambench::outputs::BaseOutput *trajectory;
        if(gt_trajectory) {
            for(auto lib : config->GetLoadedLibs()) {

                trajectory = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
                auto alignment = new slambench::outputs::AlignmentOutput("Alignment", new slambench::outputs::PoseOutputTrajectoryInterface(gt_trajectory), trajectory, alignment_method);
                alignment->SetActive(true);
                alignment->SetKeepOnlyMostRecent(true);

                auto aligned = new slambench::outputs::AlignedPoseOutput(trajectory->GetName() + " (Aligned)", alignment, trajectory);
                lib->GetOutputManager().RegisterOutput(aligned);

                // Align point cloud
                slambench::outputs::BaseOutput *pointcloud = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POINTCLOUD);
                if(pointcloud != nullptr) {
                    auto pc_aligned = new slambench::outputs::AlignedPointCloudOutput(pointcloud->GetName() + "(Aligned)", alignment, pointcloud);
                    lib->GetOutputManager().RegisterOutput(pc_aligned);
                }
            }
        }

        //***************************************************************************************
        // Start the GUI
        //***************************************************************************************
        bool stay_on = true;
        std::thread pangolin_thread(run_pangolin, &stay_on, config);
        while(ui == nullptr) ; // spin until UI is initialised

        //***************************************************************************************
        // Start the Experiment
        //***************************************************************************************
        SLAMBenchConfiguration::compute_loop_algorithm( config, &stay_on, ui);

        //***************************************************************************************
        // End and wait until the GUI stops
        //***************************************************************************************
        stay_on = false;
        pangolin_thread.join();

    } catch (const SLAMBenchException& e) {
        std::cout << "An error occurred during the execution." << std::endl;
        std::cout << e.what() << std::endl;

    }
    return 0;
}

void run_pangolin(bool *stay_on, SLAMBenchConfiguration *config) {
    std::cerr << "Creation of GUI interface." << std::endl;
    auto ui_pangolin = new SLAMBenchUI_Pangolin();
    ui_pangolin->AddOutputManager("Ground Truth", &config->GetGroundTruth());
    for(auto lib : config->GetLoadedLibs()) {
        ui_pangolin->AddOutputManager(lib->getName(), &lib->GetOutputManager());
    }
    ui_pangolin->InitialiseOutputs();

    ui = dynamic_cast<SLAMBenchUI*> (ui_pangolin);

    // Provide the current camera position to the GUI
    // FIXME : temporary values (Toky)
    float fx = 520.9000244141;
    float fy = 521.0000000000;
    float cx = 324.5999755859;
    float cy = 249.2000122070;
    ui->update_camera(480,640, fx, fy , cx , cy );

    std::cerr << "Start rendering loop." << std::endl;
    while( !pangolin::ShouldQuit()) {

        usleep(40000);
        if (!ui->process ()) {
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



	//***************************************************************************************
	// CLOSE LIBRARIES
	//***************************************************************************************

    std::cout << "End of program." << std::endl;

}

/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include <SLAMBenchAPI.h>
#include <SLAMBenchConfiguration.h>
#include <SLAMBenchUI_Pangolin.h>
#include <thread>

#include <io/InputInterface.h>
#include <io/FrameBufferSource.h>
#include <io/SLAMFrame.h>
#include <io/sensor/Sensor.h>
#include <SLAMBenchException.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <pangolin/pangolin.h>


std::string alignment_technique = "original";
std::string default_alignment_technique = "original";

void run_pangolin(bool *stay_on, SLAMBenchConfiguration *config);
static SLAMBenchUI * volatile ui = nullptr;

int main(int argc, char * argv[])
{

	try {

		SLAMBenchConfiguration * config = new SLAMBenchConfiguration();

		//***************************************************************************************
		// Start the argument processing
		//***************************************************************************************

		config->addParameter(TypedParameter<std::string> ("a",     "alignment-technique",      "Select an alignment technique by name, if not found, default used (default,new).", &alignment_technique, &default_alignment_technique));
		config->GetParameterManager().ReadArgumentsOrQuit(argc, argv);
		//***************************************************************************************
		// At this point the datasets/libraries/sensors are loaded with their arguments set.
		//***************************************************************************************


		//***************************************************************************************
		// We initialise the configuration, means to retrieve groundtruth and set the alignement
		//***************************************************************************************

		config->InitGroundtruth();

		// get GT trajectory
		auto gt_trajectory = config->GetGroundTruth().GetMainOutput(slambench::values::VT_POSE);

		slambench::outputs::TrajectoryAlignmentMethod *alignment_method;
		if(alignment_technique == "original") {
			alignment_method = new slambench::outputs::OriginalTrajectoryAlignmentMethod();
		} else if(alignment_technique == "new") {
			alignment_method = new slambench::outputs::NewTrajectoryAlignmentMethod();
		} else {
			std::cerr << "Unknown alignment method " << alignment_technique << std::endl;
			return 1;
		}

		//***************************************************************************************
		// We prepare the logging and create the global metrics
		//***************************************************************************************

		// N/A

		//***************************************************************************************
		// We init the algos now because we need their output already
		// TODO: if pose and map were by default we could init the algo much later,
		//       thus move memory metric later
		//***************************************************************************************

		config->InitAlgorithms();

		// If a ground truth is available, produce an aligned trajectory for each algorithm
		if(gt_trajectory) {
			for(auto lib : config->GetLoadedLibs()) {
				// trajectory
				slambench::outputs::BaseOutput *trajectory = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);

				// produce an alignment
				auto alignment = new slambench::outputs::AlignmentOutput("Alignment", new slambench::outputs::PoseOutputTrajectoryInterface(gt_trajectory), trajectory, alignment_method);
				alignment->SetActive(true);
				alignment->SetKeepOnlyMostRecent(true);

				auto aligned = new slambench::outputs::AlignedPoseOutput(trajectory->GetName() + " (Aligned)", alignment, trajectory);
				lib->GetOutputManager().RegisterOutput(aligned);

				// try and align a pointcloud
				slambench::outputs::BaseOutput *pointcloud = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POINTCLOUD);
				if(pointcloud != nullptr) {
					auto pc_aligned = new slambench::outputs::AlignedPointCloudOutput(pointcloud->GetName() + "(Aligned)", alignment, pointcloud);
					lib->GetOutputManager().RegisterOutput(pc_aligned);
				}
			}
		}





		//***************************************************************************************
		// We Start the GUI
		//***************************************************************************************


		bool stay_on = true;
		std::thread pangolin_thread(run_pangolin, &stay_on, config);
		while(ui == nullptr) ; // spin until UI is initialised

		//***************************************************************************************
		// We Start the Experiment
		//***************************************************************************************

		SLAMBenchConfiguration::compute_loop_algorithm( config, &stay_on, ui);

		//***************************************************************************************
		// We End and wait until the GUI stop
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
	SLAMBenchUI_Pangolin * ui_pangolin = new SLAMBenchUI_Pangolin();
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

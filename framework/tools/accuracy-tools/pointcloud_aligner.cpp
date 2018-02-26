/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include <SLAMBenchAPI.h>
#include <SLAMBenchUI_Pangolin.h>
#include <thread>
#include <io/InputInterface.h>
#include <io/FrameBufferSource.h>
#include <io/SLAMFrame.h>
#include <io/sensor/Sensor.h>

#undef HAVE_OPENNI
#include <pcl/registration/icp.h>

typedef  pcl::PointXYZ point_t;

float getScore( pcl::PointCloud<point_t>::Ptr  gt,pcl::PointCloud<point_t>::Ptr  test)
{

	std::cout << "Building tree, may take a few minutes..." << std::endl;
    pcl::search::KdTree<point_t>::Ptr tree (new pcl::search::KdTree<point_t>);
    tree->setInputCloud(gt);

    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    double totalSum = 0;

    int count = 0;

	std::cout << "Start KNN search..." << std::endl;
    for(size_t i = 0; i < test->size(); i++)
    {
    	if ((i % 500) == 0)
    		std::cout << "\r" << i << std::flush;
        tree->nearestKSearch(test->at(i), 1, pointIdxNKNSearch, pointNKNSquaredDistance);
        totalSum += sqrt(pointNKNSquaredDistance.at(0));
        count++;
    }
    std::cout << std::endl;

    return totalSum / (double)test->size();
}


void computeAlignment(std::vector<slambench::outputs::OutputManager*> outputs, slambench::outputs::AlignmentOutput *alignment) {


	std::vector<pcl::PointCloud<point_t>::Ptr> clouds ;

	for(auto output_mgr : outputs) {
		for(auto output : *output_mgr) {
			slambench::outputs::BaseOutput* bo = output.second;

			if (bo->GetType() == slambench::values::VT_POINTCLOUD) {

				std::cout << "Find one !" << std::endl;
				if(bo->GetValues().empty()) {

						std::cout << "An empty one !" << std::endl;
						continue;
					}

					const slambench::values::PointCloudValue *latest_pc = static_cast<const slambench::values::PointCloudValue*>(bo->GetValues().rbegin()->second);
					size_t size = latest_pc->GetPoints().size();
					std::cout << "With " << size << " points." << std::endl;

					pcl::PointCloud<point_t>::Ptr cloud = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>);
					for (auto point : latest_pc->GetPoints()) {
						cloud->push_back({point.X,point.Y,point.Z});
					}
					clouds.push_back(cloud);

			}
		}
	}

	if (clouds.size() != 2) {
		std::cout << "Too many point clouds founds." << std::endl;
		throw std::invalid_argument("");
	}

	auto alignment_value = (slambench::values::TypeForVT<slambench::values::VT_MATRIX>::type*)alignment->GetMostRecentValue().second;
	
	pcl::transformPointCloud(*clouds[0], *clouds[0], alignment_value->GetValue());
	std::cout << "Transformation done." << std::endl;


	std::cout << "WE NEED TO Do THE TRAJECTORy ALIGNEMENT HERE !!!!" << std::endl;
	std::cout << "Initialize first scoring." << std::endl;

	int iteration = 0;
    float value = getScore(clouds[0],clouds[1]);


	std::cout << "First score is " << value << std::endl;

    if(value < 0.05)
    {

    	std::cout << "Interesting score, we start icp." << std::endl;

        pcl::IterativeClosestPoint<point_t, point_t> icp;

        pcl::PointCloud<point_t>::Ptr aligned (new pcl::PointCloud <point_t>);

        icp.setInputSource(clouds[1]);
        icp.setInputTarget(clouds[0]);
        icp.setMaximumIterations(1);

        for(int i = 0; i < 10; i++)
        {

        	std::cout << "Align pass " << i << std::endl;
            icp.align(*aligned, icp.getFinalTransformation());
            iteration++;
        }

    	std::cout << "Final scoring " << std::endl;
        value = std::min(getScore(clouds[0],aligned), value);
    } else {
    	std::cout << "Score is too far, no ICP." << std::endl;
    }

    std::cout << value << std::endl;

    return;
}


slambench::outputs::OutputManager *LoadSLAMFile(const std::string &filename, slambench::io::FrameBufferSource &fb_src) {
	auto mgr = new slambench::outputs::OutputManager ();

	auto file = slambench::io::SLAMFile::Read(filename, fb_src);
	mgr->LoadGTOutputsFromSLAMFile(file);

	return mgr;
}

int main(int argc, char * argv[])
{
	
	//***************************************************************************************
	// Init the GUI
	//***************************************************************************************

	std::cerr << "Creation of GUI interface." << std::endl;
	SLAMBenchUI_Pangolin * ui_pangolin = new SLAMBenchUI_Pangolin();
	
	// Initialise input data
	slambench::io::SingleFrameBufferSource fb_src;
	
	// for each arg, try and load an input file
	std::vector<slambench::outputs::OutputManager*> outputs;
	
	if(argc != 3) {
		printf("Usage: %s [slam file 1] [slam file 2]\n", argv[0]);
		return 1;
	}
	
	auto slamfile_alignee_filename = argv[1];
	auto slamfile_target_filename = argv[2];
	
	auto output_mgr_alignee = LoadSLAMFile(slamfile_alignee_filename, fb_src);
	auto output_mgr_target = LoadSLAMFile(slamfile_target_filename, fb_src);
	
	outputs.push_back(output_mgr_alignee);
	outputs.push_back(output_mgr_target);
	
	// Align traj1 onto traj2
	auto alignee_traj = output_mgr_alignee->GetMainOutput(slambench::values::VT_POSE);
	auto alignee_pointcloud = (slambench::values::PointCloudValue*)output_mgr_alignee->GetMainOutput(slambench::values::VT_POINTCLOUD)->GetMostRecentValue().second;
	auto target_traj = output_mgr_target->GetMainOutput(slambench::values::VT_POSE);
	
	if(alignee_traj == nullptr || target_traj == nullptr) {
		std::cerr << "Both slamfiles must provide a pose output!";
		return 1;
	}
	

	std::cerr << "File loaded" << std::endl;

	auto alignment = new slambench::outputs::AlignmentOutput("Alignment", new slambench::outputs::PoseOutputTrajectoryInterface(target_traj), alignee_traj, new slambench::outputs::OriginalTrajectoryAlignmentMethod());
	alignment->SetKeepOnlyMostRecent(true);
	auto aligned_traj1 = new slambench::outputs::AlignedPoseOutput("Aligned", alignment, alignee_traj);
	
	output_mgr_alignee->RegisterOutput(aligned_traj1);
	
	auto partially_aligned_pc_op = new slambench::outputs::Output("Partially aligned PC", slambench::values::VT_POINTCLOUD);
	auto partially_aligned_pc = new slambench::values::PointCloudValue(*alignee_pointcloud);
	partially_aligned_pc->SetTransform(((slambench::values::TypeForVT<slambench::values::VT_MATRIX>::type*)alignment->GetMostRecentValue().second)->GetValue());
	
	partially_aligned_pc_op->SetActive(true);
	partially_aligned_pc_op->AddPoint({0,0}, partially_aligned_pc);
	
	auto ui_output_manager = new slambench::outputs::OutputManager();
	ui_output_manager->RegisterOutput(partially_aligned_pc_op);
	
	
	ui_pangolin->AddOutputManager("Traj1", output_mgr_alignee);
	ui_pangolin->AddOutputManager("Traj2", output_mgr_target);
	ui_pangolin->AddOutputManager("UI", ui_output_manager);
	
	// Initialise the UI
	ui_pangolin->InitialiseOutputs();
	
	SLAMBenchUI * ui = dynamic_cast<SLAMBenchUI*> (ui_pangolin);

	// Provide the current camera position to the GUI
	// FIXME : temporary values (Toky)
	float fx = 520.9000244141;
	float fy = 521.0000000000;
	float cx = 324.5999755859;
	float cy = 249.2000122070;
	ui->update_camera(480,640, fx, fy , cx , cy );



	//***************************************************************************************
	// GUI LOOP + GUI CLOSED
	//***************************************************************************************


	std::cerr << "Start to compute" << std::endl;

	std::thread compute_thread (computeAlignment, outputs, alignment);



	std::cerr << "Start rendering loop." << std::endl;
	while( !pangolin::ShouldQuit() ) {

		usleep(40000);
		if (!ui->process ()) {
			std::cerr << "Rendering problem." << std::endl;
			exit(1);
		}
	}
	std::cout << "Stop Pangolin..." << std::endl;
	pangolin::Quit();
	std::cout << "GUI closed ... " << std::endl;

	compute_thread.join();

	//***************************************************************************************
	// CLOSE LIBRARIES
	//***************************************************************************************

	std::cout << "End of program." << std::endl;


	return 0;
}

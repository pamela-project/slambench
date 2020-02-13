/*

 Copyright (c) 2019 Intel Corp.
 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.

 This code is licensed under the MIT License.

 */

#include "SLAMBenchConfigurationLifelong.h"
#include "TimeStamp.h"
#include <Parameters.h>
#include "sb_malloc.h"

#include <io/FrameBufferSource.h>
#include <io/openni2/ONI2FrameStream.h>
#include <io/openni2/ONI2InputInterface.h>

#include <io/openni15/ONI15FrameStream.h>
#include <io/openni15/ONI15InputInterface.h>

#include <io/InputInterface.h>
#include <io/SLAMFrame.h>
#include <io/format/PointCloud.h>
#include <io/sensor/Sensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/PointCloudSensor.h>

#include <metrics/Metric.h>
#include <metrics/ATEMetric.h>
#include <metrics/PowerMetric.h>
#include <metrics/MemoryMetric.h>

#include <values/Value.h>
#include <outputs/Output.h>

#include <boost/optional.hpp>

#include <iostream>
#include <stdexcept>

#include <iomanip>
#include <map>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <dlfcn.h>
#define LOAD_FUNC2HELPER(handle,lib,f)     *(void**)(& lib->f) = dlsym(handle,#f); const char *dlsym_error_##lib##f = dlerror(); if (dlsym_error_##lib##f) {std::cerr << "Cannot load symbol " << #f << dlsym_error_##lib##f << std::endl; dlclose(handle); exit(1);}


//TODO: (Mihai) too much duplicated code here. One option is to move LOAD_FUNC2HELPER into the SLAMBenchConfiguration header
// Need to figure out how to avoid rewriting the whole thing without breaking SLAMBenchConfiguration
void SLAMBenchConfigurationLifelong::add_slam_library(std::string so_file, std::string identifier) {

    std::cerr << "new library name: " << so_file  << std::endl;

    void* handle = dlopen(so_file.c_str(),RTLD_LAZY);

    if (!handle) {
        std::cerr << "Cannot open library: " << dlerror() << std::endl;
        exit(1);
    }

    char *start=(char *)so_file.c_str();
    char *iter = start;
    while(*iter!=0){
        if(*iter=='/')
            start = iter+1;
        iter++;
    }
    std::string libName=std::string(start);
    libName=libName.substr(3, libName.length()-14);
    auto lib_ptr = new SLAMBenchLibraryHelperLifelong (identifier, libName, this->get_log_stream(),  this->GetCurrentInputInterface());
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_init_slam_system);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_new_slam_configuration);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_update_frame);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_process_once);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_clean_slam_system);
    LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_update_outputs);
    // workaround to be compatible with benchmarks that does not implement the relocalize API
    if (dlsym(handle, "_Z13sb_relocalizeP22SLAMBenchLibraryHelper")) {
        LOAD_FUNC2HELPER(handle,lib_ptr,c_sb_relocalize);
    } else {
        std::cout << "Benchmark does not implement sb_relocalize(). Will use the default." << std::endl;
        lib_ptr->c_sb_relocalize = lib_ptr->c_sb_process_once;
    }
    this->libs.push_back(lib_ptr);


    size_t pre = slambench::memory::MemoryProfile::singleton.GetOverallData().BytesAllocatedAtEndOfFrame;
    if (!lib_ptr->c_sb_new_slam_configuration(lib_ptr)) {
        std::cerr << "Configuration construction failed." << std::endl;
        exit(1);
    }
    size_t post = slambench::memory::MemoryProfile::singleton.GetOverallData().BytesAllocatedAtEndOfFrame;
    std::cerr << "Configuration consumed " << post-pre  << " bytes" << std::endl;

    GetParameterManager().AddComponent(lib_ptr);

    std::cerr << "SLAM library loaded: " << so_file << std::endl;

}

void libs_callback(Parameter* param, ParameterComponent* caller) {

    SLAMBenchConfigurationLifelong* config = dynamic_cast<SLAMBenchConfigurationLifelong*> (caller);

    TypedParameter<std::vector<std::string>>* parameter =  dynamic_cast<TypedParameter<std::vector<std::string>>*>(param) ;

    for (std::string library_name : parameter->getTypedValue()) {


        std::string library_filename   = "";
        std::string library_identifier = "";


        auto pos = library_name.find("=");
        if (pos != std::string::npos)  {
            library_filename   = library_name.substr(0, pos);
            library_identifier = library_name.substr(pos+1);
        } else {
            library_filename = library_name;
        }
        config->add_slam_library(library_filename,library_identifier);
    }
}

void dataset_callback(Parameter* param, ParameterComponent* caller) {

    SLAMBenchConfigurationLifelong* config = dynamic_cast<SLAMBenchConfigurationLifelong*> (caller);

    if (!config) {
        std::cerr << "Extremely bad usage of the force..." << std::endl;
        std::cerr << "It happened that a ParameterComponent* can not be turned into a SLAMBenchConfiguration*..." << std::endl;
        exit(1);
    }

    TypedParameter<std::vector<std::string>>* parameter =  dynamic_cast<TypedParameter<std::vector<std::string>>*>(param) ;

    for (std::string input_name : parameter->getTypedValue()) {
        config->add_input(input_name);
        config->input_filenames.push_back(input_name);
    }
    config->init_sensors();
}

bool SLAMBenchConfigurationLifelong::add_input(std::string input_file) {

    FILE * input_desc = fopen(input_file.c_str(), "r");
    if (input_desc == nullptr) {
        throw std::logic_error( "Could not open the input file" );
    }
    this->AddInputInterface(new slambench::io::FileStreamInputInterface(input_desc, new slambench::io::SingleFrameBufferSource()));

    return true;
}


SLAMBenchConfigurationLifelong::SLAMBenchConfigurationLifelong() : SLAMBenchConfiguration(dataset_callback, libs_callback) {}

void SLAMBenchConfigurationLifelong::InitGroundtruth(bool with_point_cloud) {

    if(initialised_) {
        // return;
        // auto gt_trajectory = GetGroundTruth().GetMainOutput(slambench::values::VT_POSE);
    }
    auto interface = GetCurrentInputInterface();
    if(interface != nullptr) {
        auto gt_buffering_stream = new slambench::io::GTBufferingFrameStream(interface->GetFrames());
        input_stream_ = gt_buffering_stream;

        if(realtime_mode_) {
            std::cerr << "Real time mode enabled" << std::endl;
            input_stream_ = new slambench::io::RealTimeFrameStream(input_stream_, realtime_mult_, true);
        } else {
            std::cerr << "Process every frame mode enabled" << std::endl;
        }

        GetGroundTruth().LoadGTOutputsFromSLAMFile(interface->GetSensors(), gt_buffering_stream->GetGTFrames(), with_point_cloud);
    }

    auto gt_trajectory = GetGroundTruth().GetMainOutput(slambench::values::VT_POSE);
    if(gt_trajectory == nullptr) {
        // Warn if there is no ground truth
        std::cerr << "Dataset does not provide a GT trajectory" << std::endl;
    }



    initialised_ = true;
}


void SLAMBenchConfigurationLifelong::compute_loop_algorithm(SLAMBenchConfiguration* config, bool *, SLAMBenchUI *) {

    auto config_lifelong = dynamic_cast<SLAMBenchConfigurationLifelong*>(config);
    assert(config_lifelong->initialised_);

    for (auto lib : config_lifelong->libs) {

        auto trajectory = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
        if(trajectory == nullptr) {
            std::cerr << "Algo does not provide a main pose output" << std::endl;
            exit(1);
        }
    }

    // ********* [[ MAIN LOOP ]] *********

    unsigned int frame_count = 0;
    unsigned int bags_count = 0;
    bool ongoing = false;
    std::map<SLAMBenchLibraryHelper*, Eigen::Matrix4f> libs_trans;

    slambench::io::InputInterface* interface = config_lifelong->GetCurrentInputInterface();
    while(interface!=nullptr)
    {


        // No UI tool in Lifelong SLAM for now


        // ********* [[ LOAD A NEW FRAME ]] *********

        if(config_lifelong->input_stream_ == nullptr) {
            std::cerr << "No input loaded." << std::endl;
            break;
        }

        slambench::io::SLAMFrame * current_frame = config_lifelong->input_stream_->GetNextFrame();

        while(current_frame!= nullptr)
        {

            if (current_frame->FrameSensor->GetType()!= slambench::io::GroundTruthSensor::kGroundTruthTrajectoryType) {
            // ********* [[ NEW FRAME PROCESSED BY ALGO ]] *********

            for (auto lib : config_lifelong->libs) {
                frame_count++;

                // ********* [[ SEND THE FRAME ]] *********
                ongoing=not lib->c_sb_update_frame(lib,current_frame);

                // This algorithm hasn't received enough frames yet.
                if(ongoing) {
                    continue;
                }

                // ********* [[ PROCESS ALGO START ]] *********
                lib->GetMetricManager().BeginFrame();

                slambench::TimeStamp ts = current_frame->Timestamp;

                if (!config_lifelong->input_interface_updated) {
                    if (not lib->c_sb_process_once (lib)) {
                        std::cerr <<"Error after lib->c_sb_process_once." << std::endl;
                        exit(1);
                    }
                } else {
                    // ********** [[or relocalization]] **********
                    //Mihai: need assertion / safety mechanism to avoid ugly errors
                    bool res = dynamic_cast<SLAMBenchLibraryHelperLifelong*>(lib)->c_sb_relocalize(lib);
                    config_lifelong->input_interface_updated = false;
                    // Mihai: Might want to add a reset function to feed the pose?
                    if(!res && config_lifelong->gt_available)
                    {
                        config_lifelong->aided_reloc = true;
                        //Find the nearest one
                        auto gt_frames = dynamic_cast<slambench::io::GTBufferingFrameStream*>(config_lifelong->input_stream_)->GetGTFrames();
                        int index = 0;
                        auto gt_frame = gt_frames->GetFrame(index);
                        slambench::TimeStamp gt_ts = gt_frame->Timestamp;
                        while (gt_frame) {
                            if(gt_ts > ts) {
                                break;
                            }
                            index++;
                            gt_frame = gt_frames->GetFrame(index);
                            gt_ts = gt_frame->Timestamp;
                        }
                        Eigen::Matrix4f &t = libs_trans[lib];
                        Eigen::Matrix4f gt;
                        memcpy(gt.data(), gt_frame->GetData(), gt_frame->GetSize());
                        dynamic_cast<slambench::io::DeserialisedFrame*>(gt_frame)->getFrameBuffer().resetLock();
                        Eigen::Matrix4f es = t.inverse() * gt;
                        memcpy(gt_frame->GetData(), es.data(), gt_frame->GetSize());
                        dynamic_cast<slambench::io::DeserialisedFrame*>(gt_frame)->getFrameBuffer().resetLock();

                        lib->c_sb_update_frame(lib, gt_frame);// groundtruth feed

                        dynamic_cast<slambench::io::DeserialisedFrame*>(gt_frame)->getFrameBuffer().resetLock();
                        memcpy(gt_frame->GetData(), gt.data(), gt_frame->GetSize());
                        dynamic_cast<slambench::io::DeserialisedFrame*>(gt_frame)->getFrameBuffer().resetLock();
                    }
                }


                if(!lib->c_sb_update_outputs(lib, &ts)) {
                    std::cerr << "Failed to get outputs" << std::endl;
                    exit(1);
                }

                lib->GetMetricManager().EndFrame();
                if (libs_trans.count(lib) == 0 && config_lifelong->gt_available) {
                    libs_trans[lib] = dynamic_cast<SLAMBenchLibraryHelperLifelong*>(lib)->alignment->getTransformation();
                }
            }

            // ********* [[ FINALIZE ]] *********

            if(!ongoing) {
                config->FireEndOfFrame();
                if (config_lifelong->frame_limit) {
                    if (frame_count >= config_lifelong->frame_limit) {
                        break;
                    }
                }
            }
            }
            // we're done with the frame
            current_frame->FreeData();
            current_frame = config_lifelong->input_stream_->GetNextFrame();
        }
        std::cerr << "Last frame in bag processed." << std::endl;
        // Load next bag if it exists
        config_lifelong->LoadNextInputInterface();
        interface = config_lifelong->GetCurrentInputInterface();
        bags_count++;
        frame_count = 0;
    }

}



slambench::io::InputInterface * SLAMBenchConfigurationLifelong::GetCurrentInputInterface()
{
    if(input_interfaces.empty()) {
        return nullptr; // FIXME: (Mihai) this looks dodgy
        //throw std::logic_error("Input interface has not been added to SLAM configuration");
    }
    return input_interfaces.front();
}

const slambench::io::SensorCollection& SLAMBenchConfigurationLifelong::GetSensors()
{
    return this->GetCurrentInputInterface()->GetSensors();
}


void SLAMBenchConfigurationLifelong::AddInputInterface(slambench::io::InputInterface *input_ref) {
    //workaround to be compatible with benchmarks that does not implement sensors resetting.
    //assume different input_interfaces has exactly the same types of sensors.
    //If sensors are different, may introduce problems.
    if (input_interfaces.empty()) {
        first_sensors = &input_ref->GetSensors();
    } else {
        input_ref->GetSensors() = *first_sensors;
    }
    input_interfaces.push_back(input_ref);
}

void SLAMBenchConfigurationLifelong::init_sensors() {
    for (slambench::io::Sensor *sensor : this->GetCurrentInputInterface()->GetSensors()) {
        GetParameterManager().AddComponent(dynamic_cast<ParameterComponent*>(&(*sensor)));
    }
}

void SLAMBenchConfigurationLifelong::LoadNextInputInterface() {
    input_interfaces.pop_front();
    reset_sensors();
    if(input_interfaces.empty())
        return;

    init_sensors();
    InitGroundtruth();
    init_cw();
    for (auto lib : this->libs) {
        lib->updateInputInterface(this->GetCurrentInputInterface());
        dynamic_cast<SLAMBenchLibraryHelperLifelong*>(lib)->resetSensorUpdate(true);
    }
    input_interface_updated = true;
}

// ---- get cpu info ---- //
void getCPUInfo(std::string& cpuInfo)
{
    std::string::size_type position;
    FILE *fp = fopen("/proc/cpuinfo", "r");
    if(NULL == fp)
        std::cerr << "failed to open cpuinfo\n";
    char szTest[1000] = {0};
    while(!feof(fp))
    {
        memset(szTest, 0, sizeof(szTest));
        fgets(szTest, sizeof(szTest) - 1, fp);
        cpuInfo = szTest;
        position = cpuInfo.find("model name");
        if (position != cpuInfo.npos && position == 0) {
            position = cpuInfo.find(":");
            cpuInfo = cpuInfo.substr(position + 1, 100);
            position = cpuInfo.find("\n");
            if (position != cpuInfo.npos) {
                cpuInfo = cpuInfo.substr(0, position);
            }
            break;
        }
    }
    fclose(fp);
}


// ---- get memory info ---- //
void getMemoryInfo(int& mem)
{
    std::string memInfo;
    std::string::size_type position;
    FILE *fp = fopen("/proc/meminfo", "r");
    if(NULL == fp)
        std::cerr << "failed to open meminfo\n";
    char szTest[1000] = {0};
    while(!feof(fp))
    {
        memset(szTest, 0, sizeof(szTest));
        fgets(szTest, sizeof(szTest) - 1, fp);
        memInfo = szTest;
        position = memInfo.find("MemTotal");
        if (position != memInfo.npos && position == 0) {
            position = memInfo.find(":");
            memInfo = memInfo.substr(position + 1, 100);
            position = 0;
            for(size_t i = 0; i < memInfo.size(); i++) {
                if (int(memInfo[i]) >= 48 && int(memInfo[i]) <= 57) {
                    break;
                }
                position++;
            }
            memInfo = memInfo.substr(position, 100);
            position = 0;
            for(size_t i = 0; i < memInfo.size(); i++) {
                if (int(memInfo[i]) < 48 || int(memInfo[i]) > 57) {
                    break;
                }
                position++;
            }
            memInfo = memInfo.substr(0, position);
            mem = std::stoi(memInfo);
            break;
        }
    }
    fclose(fp);
}

void SLAMBenchConfigurationLifelong::init_cw() {

    if (cw_initialised_) {
        this->OutputToTxt();
        aided_reloc = false;
        // delete memory_metric;
		// delete duration_metric;
		// delete power_metric;
        delete cw;
        cw = nullptr;
        // memory_metric = nullptr;
        // duration_metric = nullptr;
        // power_metric = nullptr;
    }
    auto gt_traj = this->GetGroundTruth().GetMainOutput(slambench::values::VT_POSE);
    slambench::outputs::TrajectoryAlignmentMethod *alignment_method;
	if(alignment_technique_ == "original") {
		alignment_method = new slambench::outputs::OriginalTrajectoryAlignmentMethod();
	} else if(alignment_technique_ == "new") {
		alignment_method = new slambench::outputs::NewTrajectoryAlignmentMethod();
	} else {
	std::cerr << "Unknown alignment method " << alignment_technique << std::endl;
	throw std::logic_error("Unknown alignment method");
	}
    cw = new slambench::ColumnWriter(this->get_log_stream(), "\t");

	cw->AddColumn(&(this->row_number));
    bool have_timestamp = false;
    slambench::metrics::MemoryMetric* memory_metric = new slambench::metrics::MemoryMetric();
    gpuInfo = memory_metric->cuda_monitor.IsActive() ? memory_metric->cuda_monitor.device_name : "";
    if (!cw_initialised_) {
	    duration_metric = new slambench::metrics::DurationMetric();
	    power_metric    = new slambench::metrics::PowerMetric();
    }

    for(SLAMBenchLibraryHelper *lib : this->GetLoadedLibs()) {
        if (cw_initialised_) {
            lib->GetMetricManager().reset();
            lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE)->reset();
        }

	    // retrieve the trajectory of the lib
		auto lib_traj = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
		if (lib_traj == nullptr) {
			std::cerr << "There is no output trajectory in the library outputs." << std::endl;
			exit(1);
		}

		// Create timestamp column if we don't have one
		if(!have_timestamp) {
			have_timestamp = true;
			cw->AddColumn(new slambench::OutputTimestampColumnInterface(lib_traj));
		}

		if (gt_traj) {
            gt_available = true;
			// Create an aligned trajectory
			dynamic_cast<SLAMBenchLibraryHelperLifelong*>(lib)->alignment = new slambench::outputs::AlignmentOutput("Alignment", new slambench::outputs::PoseOutputTrajectoryInterface(gt_traj), lib_traj, alignment_method);
			dynamic_cast<SLAMBenchLibraryHelperLifelong*>(lib)->alignment->SetActive(true);
			dynamic_cast<SLAMBenchLibraryHelperLifelong*>(lib)->alignment->SetKeepOnlyMostRecent(true);
			auto aligned = new slambench::outputs::AlignedPoseOutput(lib_traj->GetName() + " (Aligned)", dynamic_cast<SLAMBenchLibraryHelperLifelong*>(lib)->alignment, lib_traj);

			// Add ATE metric
			auto ate_metric = new slambench::metrics::ATEMetric(new slambench::outputs::PoseOutputTrajectoryInterface(aligned), new slambench::outputs::PoseOutputTrajectoryInterface(gt_traj));
			if (ate_metric->GetValueDescription().GetStructureDescription().size() > 0) {
				lib->GetMetricManager().AddFrameMetric(ate_metric);
				cw->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, ate_metric, lib->GetMetricManager().GetFramePhase()));
			}

			// Add RPE metric
			auto rpe_metric = new slambench::metrics::RPEMetric(new slambench::outputs::PoseOutputTrajectoryInterface(aligned), new slambench::outputs::PoseOutputTrajectoryInterface(gt_traj));
			lib->GetMetricManager().AddFrameMetric(rpe_metric);
			cw->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, rpe_metric, lib->GetMetricManager().GetFramePhase()));
		} else {
            gt_available = false;
        }

        // Add a duration metric
		lib->GetMetricManager().AddFrameMetric(duration_metric);
		lib->GetMetricManager().AddPhaseMetric(duration_metric);
		cw->AddColumn(new slambench::ValueLibColumnInterface(lib, duration_metric, lib->GetMetricManager().GetFramePhase()));
		for(auto phase : lib->GetMetricManager().GetPhases()) {
			cw->AddColumn(new slambench::ValueLibColumnInterface(lib, duration_metric, phase));
		}

		// Add a memory metric
		lib->GetMetricManager().AddFrameMetric(memory_metric);
		cw->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, memory_metric, lib->GetMetricManager().GetFramePhase()));

		// Add a power metric if it makes sense
		if (power_metric->GetValueDescription().GetStructureDescription().size() > 0) {
			lib->GetMetricManager().AddFrameMetric(power_metric);
			cw->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, power_metric, lib->GetMetricManager().GetFramePhase()));
		}

		// Add XYZ row from the trajectory
		auto traj = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
		traj->SetActive(true);
		cw->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, new slambench::outputs::PoseToXYZOutput(traj)));

	}


    frame_callbacks_.clear();
	this->AddFrameCallback([this]{this->cw->PrintRow();}); // @suppress("Invalid arguments")
    cw_initialised_ = true;

	cw->PrintHeader();

}

void SLAMBenchConfigurationLifelong::OutputToTxt()
{
    if (this->output_filename_ == "" ) {
        return;
    }
    std::string input_name = this->input_filenames[this->count];
    std::size_t found = input_name.find_last_of("/");
    std::string scene = input_name.substr(found + 1, 20);
    std::size_t found2 = scene.find("-");
    std::size_t found3 = scene.find("-", found2 + 1);
    std::string seq = scene.substr(found3 + 1, 1);
    scene = scene.substr(0, found2);

    if (this->count == 0) {
        std::string cpuInfo;
        int memInfo;
        getCPUInfo(cpuInfo);
        getMemoryInfo(memInfo);
        for(SLAMBenchLibraryHelper *lib : this->GetLoadedLibs()) {
            std::ofstream OutFile;
		    OutFile.open(this->output_filename_ + "_" + lib->get_library_name() + ".txt", std::ios::out);
            OutFile << "scene: " << scene << std::endl;
            OutFile << "algorithm: " <<  lib->get_library_name() <<std::endl;
            OutFile << "topics: " << std::endl;
            OutFile << "frame: " << std::endl;
            OutFile << "CPU: " << cpuInfo << std::endl;
            OutFile << "GPU: " << gpuInfo << std::endl;
            OutFile << "memory: " << memInfo / 1024 / 1024 << " GB"<< std::endl << std::endl;
            OutFile.close();
        }
    }
	float x, y, z;
	Eigen::Matrix3d R;
//	struct timeval tv;
	for(SLAMBenchLibraryHelper *lib : this->GetLoadedLibs()) {
	    std::ofstream OutFile;
		OutFile.open(this->output_filename_ + "_" + lib->get_library_name() + ".txt", std::ios::app);
        OutFile <<  "seq: " << seq <<std::endl;
        OutFile << "aided_reloc: " << aided_reloc << std::endl;
   		//OutFile << "#DatasetTimestamp, position.x, y, z, quaterniond.x, y, z, w"<<std::endl;
		auto output = lib->GetOutputManager().GetOutput("Pose")->GetValues();
		auto duration = lib->GetMetricManager().getDuration();
		auto du = duration.begin();
        auto it = output.begin();
        auto pose_previous = (dynamic_cast<const slambench::values::PoseValue*>(it->second))->GetValue();
		for (; it != output.end(); ++it, ++du ) {
            if (pose_previous != (dynamic_cast<const slambench::values::PoseValue*>(it->second))->GetValue()||it == output.begin()){
			    x = (dynamic_cast<const slambench::values::PoseValue*>(it->second))->GetValue()(0, 3);
                if (std::to_string(x) == "-nan") {
                    continue;
                }
			    y = (dynamic_cast<const slambench::values::PoseValue*>(it->second))->GetValue()(1, 3);
			    z = (dynamic_cast<const slambench::values::PoseValue*>(it->second))->GetValue()(2, 3);
                for (int i = 0; i < 3; i++) {
				    for (int j = 0; j < 3; j++) {
					    R(i, j) = (dynamic_cast<const slambench::values::PoseValue*>(it->second))->GetValue()(i, j);
				    }
			    }
			    Eigen::Quaterniond q = Eigen::Quaterniond(R);
    		    q.normalize();
//			    gettimeofday(&tv,NULL);
			    OutFile<<it->first<<" "<<*du<<" "<<x<<" "<<y<<" "<<z<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<std::endl;
                pose_previous = (dynamic_cast<const slambench::values::PoseValue*>(it->second))->GetValue();
            }
		}
        OutFile<<std::endl;
		OutFile.close();
	}
    this->count++;
}


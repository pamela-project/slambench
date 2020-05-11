/*

 Copyright (c) 2019 Intel Corp., University of Manchester.

 This code is licensed under the MIT License.

 */

#include "SLAMBenchConfigurationLifelong.h"
#include "TimeStamp.h"
#include <Parameters.h>
#include "sb_malloc.h"
#include "ResultWriter.h"

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
#include <boost/filesystem.hpp>

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
void SLAMBenchConfigurationLifelong::AddLibrary(std::string so_file, std::string identifier) {

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
    auto lib_ptr = new SLAMBenchLibraryHelper (identifier, libName, this->GetLogStream(),  this->GetCurrentInputInterface());
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
        config->AddLibrary(library_filename,library_identifier);
    }
}

void dataset_callback(Parameter* param, ParameterComponent* caller) {

    auto config = dynamic_cast<SLAMBenchConfigurationLifelong*> (caller);

    if (!config) {
        std::cerr << "Extremely bad usage of the force..." << std::endl;
        std::cerr << "It happened that a ParameterComponent* can not be turned into a SLAMBenchConfiguration*..." << std::endl;
        exit(1);
    }

    auto* parameter =  dynamic_cast<TypedParameter<std::vector<std::string>>*>(param);

    for (const std::string& input_name : parameter->getTypedValue()) {
        config->AddInput(input_name);
    }
    config->InitSensors();
}

bool SLAMBenchConfigurationLifelong::AddInput(const std::string& input_file) {

    FILE * input_desc = fopen(input_file.c_str(), "r");
    if (input_desc == nullptr) {
        throw std::logic_error( "Could not open the input file" );
    }
    AddInputInterface(new slambench::io::FileStreamInputInterface(input_desc, new slambench::io::SingleFrameBufferSource()));
    input_filenames_.push_back(input_file);
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

void SLAMBenchConfigurationLifelong::InitAlignment() {
    slambench::outputs::TrajectoryAlignmentMethod *alignment_method;
    if(alignment_technique_ == "original") {
        alignment_method = new slambench::outputs::OriginalTrajectoryAlignmentMethod();
    } else if(alignment_technique_ == "new") {
        alignment_method = new slambench::outputs::NewTrajectoryAlignmentMethod();
    } else if(alignment_technique_ == "umeyama") {
        alignment_method = new slambench::outputs::UmeyamaTrajectoryAlignmentMethod();
    } else {
        std::cerr << "Unknown alignment method " << alignment_technique << std::endl;
        throw std::logic_error("Unknown alignment method");
	}

    auto gt_traj = GetGroundTruth().GetMainOutput(slambench::values::VT_POSE);

    assert(GetLoadedLibs().size() == 1); // following code cannot work with multiple libs
    SLAMBenchLibraryHelper *lib = GetLoadedLibs()[0];
	auto lib_traj = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
    alignment_.reset(new slambench::outputs::AlignmentOutput("Alignment", new slambench::outputs::PoseOutputTrajectoryInterface(gt_traj), lib_traj, alignment_method));
    alignment_->SetActive(true);
    alignment_->SetKeepOnlyMostRecent(true);
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

    unsigned int input_seq = 0;
    bool ongoing = false;
    std::map<SLAMBenchLibraryHelper*, Eigen::Matrix4f> libs_trans;

    while(true)
    {
        unsigned int frame_count = 0;
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

                if (!config_lifelong->input_interface_updated_) {
                    if (not lib->c_sb_process_once (lib)) {
                        std::cerr <<"Error after lib->c_sb_process_once." << std::endl;
                        exit(1);
                    }
                } else {
                    // ********** [[or relocalization]] **********
                    //Mihai: need assertion / safety mechanism to avoid ugly errors
                    bool res = lib->c_sb_relocalize(lib);
                    config_lifelong->input_interface_updated_ = false;
                    /* If the library failed to re-localize at the beginning of a new input,
                       the framework will send a ground-truth pose to it (so-called aided_reloc).
                       The sent pose is transformed to be relative to the first estimated pose
                       from the library. */
                    // Mihai: Might want to add a reset function to feed the pose?
                    if(!res && config_lifelong->gt_available_)
                    {
                        config_lifelong->aided_reloc_ = true;
                        //Find the nearest one
                        auto gt_frame = dynamic_cast<slambench::io::GTBufferingFrameStream*>(config_lifelong->input_stream_)->GetGTFrames()->GetClosestFrameToTime(ts);
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
                if (libs_trans.count(lib) == 0 && config_lifelong->gt_available_) {
                    libs_trans[lib] = config_lifelong->alignment_->getTransformation();
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
        if (!config_lifelong->output_filename_.empty()) config_lifelong->SaveResults();
        // Load next input if there be
        if (!config_lifelong->LoadNextInputInterface()) break;
        // Freeze the alignment after end of the first input
        if (input_seq++ == 0) config_lifelong->alignment_->SetFreeze(true);
    }
}

slambench::io::InputInterface * SLAMBenchConfigurationLifelong::GetCurrentInputInterface()
{
    if(input_interfaces_.empty()) {
        throw std::logic_error("Input interface has not been added to SLAM configuration");
    }
    return input_interfaces_.front();
}

const slambench::io::SensorCollection& SLAMBenchConfigurationLifelong::GetSensors()
{
    return this->GetCurrentInputInterface()->GetSensors();
}

void SLAMBenchConfigurationLifelong::AddInputInterface(slambench::io::InputInterface *input_ref) {
    //workaround to be compatible with benchmarks that does not implement sensors resetting.
    //assume different input_interfaces_ has exactly the same types of sensors.
    //If sensors are different, may introduce problems.
    if (input_interfaces_.empty()) {
        first_sensors_ = &input_ref->GetSensors();
    } else {
        input_ref->GetSensors() = *first_sensors_;
    }
    input_interfaces_.push_back(input_ref);
}

void SLAMBenchConfigurationLifelong::InitSensors() {
    for (slambench::io::Sensor *sensor : this->GetCurrentInputInterface()->GetSensors()) {
        GetParameterManager().AddComponent(dynamic_cast<ParameterComponent*>(&(*sensor)));
    }
}

bool SLAMBenchConfigurationLifelong::LoadNextInputInterface() {
    input_interfaces_.pop_front();
    ResetSensors();
    if(input_interfaces_.empty())
        return false;

    InitSensors();
    InitGroundtruth();
    InitWriter();
    for (auto lib : this->libs) {
        lib->update_input_interface(this->GetCurrentInputInterface());
    }
    input_interface_updated_ = true;
    current_input_id_++;
    return true;
}

void SLAMBenchConfigurationLifelong::InitWriter() {

    if (writer_) {
        for(SLAMBenchLibraryHelper *lib : this->GetLoadedLibs()) {
            lib->GetMetricManager().reset();
            lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE)->reset();
        }
        aided_reloc_ = false;
    } else {
        // the following metrics last for all inputs; other metrics are allocated for only one input
	    duration_metric_ = std::make_shared<slambench::metrics::DurationMetric>();
	    power_metric_ = std::make_shared<slambench::metrics::PowerMetric>();
    }
    auto gt_traj = this->GetGroundTruth().GetMainOutput(slambench::values::VT_POSE);
    if (!alignment_) InitAlignment();

    writer_.reset(new slambench::ColumnWriter(this->GetLogStream(), "\t"));

	writer_->AddColumn(&(this->row_number_));
    bool have_timestamp = false;
    auto memory_metric = std::make_shared<slambench::metrics::MemoryMetric>();

    for(SLAMBenchLibraryHelper *lib : this->GetLoadedLibs()) {

	    // retrieve the trajectory of the lib
		auto lib_traj = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
		if (lib_traj == nullptr) {
			std::cerr << "There is no output trajectory in the library outputs." << std::endl;
			exit(1);
		}

		// Create timestamp column if we don't have one
		if(!have_timestamp) {
			have_timestamp = true;
			writer_->AddColumn(new slambench::OutputTimestampColumnInterface(lib_traj));
		}

		if (gt_traj) {
            gt_available_ = true;
			// Create an aligned trajectory
			auto aligned = new slambench::outputs::AlignedPoseOutput(lib_traj->GetName() + " (Aligned)", &*alignment_, lib_traj);

			// Add ATE metric
			auto ate_metric = std::make_shared<slambench::metrics::ATEMetric>(new slambench::outputs::PoseOutputTrajectoryInterface(aligned), new slambench::outputs::PoseOutputTrajectoryInterface(gt_traj));
			if (ate_metric->GetValueDescription().GetStructureDescription().size() > 0) {
				lib->GetMetricManager().AddFrameMetric(ate_metric);
				writer_->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, &*ate_metric, lib->GetMetricManager().GetFramePhase()));
			}

			// Add RPE metric
			auto rpe_metric = std::make_shared<slambench::metrics::RPEMetric>(new slambench::outputs::PoseOutputTrajectoryInterface(aligned), new slambench::outputs::PoseOutputTrajectoryInterface(gt_traj));
			lib->GetMetricManager().AddFrameMetric(rpe_metric);
			writer_->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, &*rpe_metric, lib->GetMetricManager().GetFramePhase()));
		} else {
            gt_available_ = false;
        }

        // Add a duration metric
		lib->GetMetricManager().AddFrameMetric(duration_metric_);
		lib->GetMetricManager().AddPhaseMetric(duration_metric_);
		writer_->AddColumn(new slambench::ValueLibColumnInterface(lib, &*duration_metric_, lib->GetMetricManager().GetFramePhase()));
		for(auto phase : lib->GetMetricManager().GetPhases()) {
			writer_->AddColumn(new slambench::ValueLibColumnInterface(lib, &*duration_metric_, phase));
		}

		// Add a memory metric
		lib->GetMetricManager().AddFrameMetric(memory_metric);
		writer_->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, &*memory_metric, lib->GetMetricManager().GetFramePhase()));

		// Add a power metric if it makes sense
		if (power_metric_->GetValueDescription().GetStructureDescription().size() > 0) {
			lib->GetMetricManager().AddFrameMetric(power_metric_);
			writer_->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, &*power_metric_, lib->GetMetricManager().GetFramePhase()));
		}

		// Add XYZ row from the trajectory
		auto traj = lib->GetOutputManager().GetMainOutput(slambench::values::VT_POSE);
		traj->SetActive(true);
		writer_->AddColumn(new slambench::CollectionValueLibColumnInterface(lib, new slambench::outputs::PoseToXYZOutput(traj)));

	}


    frame_callbacks_.clear();
	this->AddFrameCallback([this]{writer_->PrintRow();}); // @suppress("Invalid arguments")

	writer_->PrintHeader();

}

void SLAMBenchConfigurationLifelong::SaveResults()
{
    if (output_filename_ == "" ) {
        return;
    }
    boost::filesystem::path input_name(input_filenames_[current_input_id_]);
    input_name = input_name.filename();
    boost::filesystem::path output_name = boost::filesystem::path(output_filename_);
    boost::filesystem::path output_prefix;
    boost::filesystem::path gt_dir;
    if (boost::filesystem::is_directory(output_name)) {
        output_name.append("/");
        output_prefix = output_name;
        gt_dir = output_name;
    } else {
        output_prefix = output_name.replace_extension().append("-");
        gt_dir = output_name.parent_path();
    }
    boost::filesystem::path gt_file = gt_dir;
    gt_file += input_name;
    gt_file.replace_extension(".gt");

    bool first_input = current_input_id_ == 0;
    static std::string cpu_info = ResultWriter::GetCPUModel();
    static std::string gpu_info = "";// memory_metric->cuda_monitor.IsActive() ? memory_metric->cuda_monitor.device_name : "";
    static std::string mem_info = ResultWriter::GetMemorySize();

    for(SLAMBenchLibraryHelper *lib : this->GetLoadedLibs()) {
        std::string filename = output_prefix.append(lib->get_library_name() + ".txt").string();
        std::ofstream out(filename, first_input ? std::ios::out : std::ios::app);
        ResultWriter writer(out);
        if (first_input) {
            writer.WriteKV("benchmark", lib->get_library_name());
            writer.WriteKV("inputs", input_filenames_);
            writer.WriteKV("CPU", cpu_info);
            if (!gpu_info.empty()) writer.WriteKV("GPU", gpu_info);
            writer.WriteKV("memory", mem_info);
        }
        out << std::endl;
        writer.WriteKV("input", input_name.string());
        writer.WriteKV("aided_reloc", std::to_string(aided_reloc_));
        slambench::outputs::BaseOutput::value_map_t traj = lib->GetOutputManager().GetOutput("Pose")->GetValues();
        writer.WriteTrajectory(traj);
        std::cout << "Results saved into " << filename << std::endl;
    }

    if (!boost::filesystem::exists(gt_file)) {
        std::ofstream out(gt_file.string());
        slambench::outputs::BaseOutput::value_map_t traj = GetGroundTruth().GetMainOutput(slambench::values::VT_POSE)->GetValues();
        ResultWriter writer(out);
        writer.WriteKV("input", input_name.string());
        writer.WriteTrajectory(traj);
        std::cout << "Ground-truth saved into " << gt_file.string() << std::endl;
    }
}


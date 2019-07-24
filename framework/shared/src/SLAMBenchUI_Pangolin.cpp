/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include <SLAMBenchUI_Pangolin.h>
#include <SLAMBenchConfiguration.h>
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

SLAMBenchUI_Pangolin::SLAMBenchUI_Pangolin() : SLAMBenchUI() {

	isFreeRunning = false;

	int view_width = 1280;
	int view_height = 980;
	int panel_width = 360;

	view_width += panel_width;

	pangolin::Params windowParams;

	windowParams.Set("SAMPLE_BUFFERS", 0);
	windowParams.Set("SAMPLES", 0);

	pangolin::CreateWindowAndBind("Main", view_width, view_height, windowParams);



	glGenTextures(MAX_WINDOW, textureId);
	glGenBuffers(1, &vbo);

	// Define Camera Render Object (for view / scene browsing)
	pangolin::OpenGlMatrix proj = pangolin::ProjectionMatrix(640,480,420,420,320,240,0.1,1000);
	s_cam = pangolin::OpenGlRenderState(proj,   pangolin::ModelViewLookAt(-1, -5, -1, 0, 0, 0, pangolin::AxisNegY));

	//! Set bounds for the View using mixed fractional / pixel coordinates (OpenGl view coordinates)
	//  View& SetBounds(Attach bottom, Attach top, Attach left, Attach right);



	// SetBounds (Attach bottom, Attach top, Attach left, Attach right, double aspect);

	// 'Negative' aspect ratio causes pangolin to overdraw and then trim the 
	// view. This is a little bit hacky but is a much easier solution than
	// trapping window resize events and rescaling the window then.
	pangolin::Display("cam").SetBounds(0.0, 1.0f, 0.0f, 1.0f, 640 / 480.0).SetHandler(new pangolin::Handler3D(s_cam))
					.SetLock(pangolin::LockRight, pangolin::LockTop);

	pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(panel_width));

	frameCount = new pangolin::Var<long long>("ui.Frame", 0);
	TVM = new pangolin::Var<long long>("ui.Total Virtual Memory (MB)", 0);
	CVM = new pangolin::Var<long long>("ui.Used Virtual Memory (MB)", 0);
	VSIZE = new pangolin::Var<long long>("ui.Process Virtual Memory (MB)", 0);
	TPM = new pangolin::Var<long long>("ui.Total Physical Memory (MB)", 0);
	CPM = new pangolin::Var<long long>("ui.Used  Physical Memory (MB)", 0);
	VRSS = new pangolin::Var<long long>("ui.Process Physical Memory (MB)", 0);


	showEveryPose = new pangolin::Var<int>("ui.Show every poses", 0, 0, 100);

	if(CanFreeRun()) {
		freeRunning = new pangolin::Var<bool>("ui.Free Running", true, true);
	}

	if(CanStep()) {
		nextFrame = new pangolin::Var<bool>("ui.Next Frame", false, false);
	}
	unFollow=NULL;

}

SLAMBenchUI_Pangolin::~SLAMBenchUI_Pangolin() {
	if(unFollow)
		delete unFollow;
	delete frameCount;
	delete VRSS ;
	delete VSIZE;
	delete TVM  ;
	delete CVM  ;
	delete TPM  ;
	delete CPM  ;

}

void SLAMBenchUI_Pangolin::AddFollowControls()
{

	pangolin::Var<bool> categoryText("ui.description.Camera Pose", false,false);

	for(auto output_mgr : GetOutputManagers()) {
		const std::string &prefix = output_mgr.first;
		for(auto output : *output_mgr.second) {
			slambench::outputs::BaseOutput* bo = output.second;
			std::string panelname = "ui.output." + prefix + "." + bo->GetName();

			switch(bo->GetType()) {
			case slambench::values::VT_POSE:
				outputs_follow_.insert({bo, new pangolin::Var<bool>(panelname + "." + bo->GetName() + " Follow", false, true)});
				break;
			default:
				break;
			}
		}
	}
}

void SLAMBenchUI_Pangolin::AddBackgroundControls()
{

	pangolin::Var<bool> categoryText("ui.description.Backgrounds", false,false);

	for(auto output_mgr : GetOutputManagers()) {
		const std::string &prefix = output_mgr.first;

		for(auto output : *output_mgr.second) {
			slambench::outputs::BaseOutput* bo = output.second;
			std::string panelname = "ui.output." + prefix + "." + bo->GetName();

			switch(bo->GetType()) {
			case slambench::values::VT_FRAME:
				outputs_background_.insert({bo, new pangolin::Var<bool>(panelname + "." + bo->GetName() + " Background", false, true)});
				break;
			default:
				break;
			}
		}
	}

}

void SLAMBenchUI_Pangolin::AddControlsForOutput(const std::string &prefix, slambench::outputs::BaseOutput* output)
{



	std::string button_name = "ui.output." + prefix + "." + output->GetName() ;


	if(output->GetType() == slambench::values::VT_STRING) {
		outputs_text_[output] = new pangolin::Var<std::string>(button_name);
		outputs_enabled_.insert({output, new pangolin::Var<bool>(button_name + " Enabled", true, true)});
	} else {
		outputs_enabled_.insert({output, new pangolin::Var<bool>(button_name + " Enabled", true, true)});
		outputs_visible_.insert({output, new pangolin::Var<bool>(button_name + " Visible", true, true)});

		switch(output->GetType()) {
		case slambench::values::VT_POINTCLOUD:
		case slambench::values::VT_POSE:
			outputs_colour_.insert({output, new pangolin::Var<double>(button_name + " Colour", 0.0, 0.0, 1.0)});
			break;
		default:
			break;
		}
	}
}

void SLAMBenchUI_Pangolin::InitialiseOutputs()
{
	AddBackgroundControls();
	AddFollowControls();

	for(auto output_mgr : GetOutputManagers()) {
		for(auto output : *output_mgr.second) {
			AddControlsForOutput(output_mgr.first, output.second);
		}
	}

	int i = 0;
	for(auto output : outputs_colour_) {
		pangolin::Var<double> *var = output.second;
		float value = (1.0 / outputs_colour_.size()) * (i+1);
		*var = value;
		i++;
		std::cerr << "Set " << output.first->GetName() << " to " << *var << std::endl;
	}
}

bool SLAMBenchUI_Pangolin::CanFreeRun() {
	return true;
}

bool SLAMBenchUI_Pangolin::CanStep() {
	return true;
}

bool SLAMBenchUI_Pangolin::ClearFreeRunning() {
	if(isFreeRunning) {
		isFreeRunning = false;
		std::unique_lock<std::mutex> lck(step_mutex);
		step_cv.wait(lck);
	}
	return true;
}

bool SLAMBenchUI_Pangolin::IsFreeRunning() {
	return isFreeRunning;
}

bool SLAMBenchUI_Pangolin::SetFreeRunning() {
	if(!isFreeRunning) {
		isFreeRunning = true;
		std::unique_lock<std::mutex> lck(step_mutex);
		step_cv.wait(lck);
	}
	return true;
}

bool SLAMBenchUI_Pangolin::WaitForFrame() {
	if(!CanStep()) {
		throw std::exception();
	}
	std::unique_lock<std::mutex> lck(step_mutex);
	step_cv.wait(lck);

	return true;
}



void SLAMBenchUI_Pangolin::StopTracking () {
	if(unFollow)
		unFollow->Ref().Set(true);
}

float ConvertChannel(float f) {
	// f ranges from -1 to 1, we want 0 to 1
	return (f + 1) / 2;
}

std::tuple<float,float,float> ConvertColour(std::tuple<float,float,float> incol) {
	return std::make_tuple(ConvertChannel(std::get<0>(incol)), ConvertChannel(std::get<1>(incol)), ConvertChannel(std::get<2>(incol)));
}

std::tuple<float,float,float> GetUnconvertedColour(float f) {
	// convert to wavelength
	float wl = 380 + (f * (645-380));

	if(wl >= 380 && wl <= 440) {
		return std::make_tuple(-1 * (wl - 440) / (440 - 380), 0, 1);
	} else if(wl >= 440 && wl <= 490) {
		return std::make_tuple(0, ((wl - 440) / (490 - 440)), 1);
	} else if(wl >= 490 && wl <= 510) {
		return std::make_tuple(0, 1, -1 * (wl - 510) / (510-490));
	} else if(wl >=510 && wl <= 580) {
		return std::make_tuple((wl-510) / (580-510), 1, 0);
	} else if(wl >= 580 && wl <= 645) {
		return std::make_tuple(1, -1 * (wl - 645) / (645 - 580), 0);
	} else if(wl >= 645 && wl <= 780) {
		return std::make_tuple(1, 0, 0);
	}

	return std::make_tuple(0,0,0);
}

std::tuple<float,float,float> GetColour(float f) {
	return ConvertColour(GetUnconvertedColour(f));
}

class PoseOutputDrawContext {
public:
	PoseOutputDrawContext(slambench::outputs::BaseOutput *output) : interface(output) {}

	slambench::outputs::PoseOutputTrajectoryInterface interface;
};

bool SLAMBenchUI_Pangolin::DrawPoseOutput(slambench::outputs::BaseOutput* output)
{
	assert(output->GetType() == slambench::values::VT_POSE);

	auto &context = pose_output_draw_contexts_[output];
	if(context == nullptr) {
		context = new PoseOutputDrawContext(output);
	}

	auto colour = GetColour(outputs_colour_.at(output)->Get());
	drawTrajectory(&context->interface, std::get<0>(colour), std::get<1>(colour), std::get<2>(colour), showEveryPose->Ref().Get());
	return true;
}

bool SLAMBenchUI_Pangolin::DrawPointCloudOutput(slambench::outputs::BaseOutput* output)
{
	assert(output->GetType() == slambench::values::VT_POINTCLOUD);
	output->SetKeepOnlyMostRecent(true);
	if(!output->IsActive() or output->GetValues().empty()) {
		return true;
	}

#ifdef HARRY_SPEEDUP

	auto timestamp = output->GetMostRecentValue().first;
	const slambench::values::PointCloudValue *latest_pc = static_cast<const slambench::values::PointCloudValue*>(output->GetMostRecentValue().second);
	size_t size = latest_pc->GetPoints().size();

	auto colour = GetColour(outputs_colour_.at(output)->Get());

	bool new_output = false;
	if(!pointcloud_state_.count(output)) {
		glCreateBuffers(1, &pointcloud_state_[output].VBO);
		new_output = true;
	}

	auto &state = pointcloud_state_[output];
	glBindBuffer(GL_ARRAY_BUFFER, state.VBO);
	if(new_output || state.timestamp != timestamp) {
		glBufferData(GL_ARRAY_BUFFER, sizeof(slambench::values::Point3DF) * size, latest_pc->GetPoints().data(), GL_STATIC_DRAW);
		state.timestamp = timestamp;
	}

	glColor3f(std::get<0>(colour),std::get<1>(colour),std::get<2>(colour));

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMultMatrixf((GLfloat*)latest_pc->GetTransform().data());

	glVertexPointer(3, GL_FLOAT, sizeof(slambench::values::Point3DF), 0);

	glEnableClientState(GL_VERTEX_ARRAY);

	glDrawArrays(GL_POINTS, 0, size);

	glDisableClientState(GL_VERTEX_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glPopMatrix();

	return true;
#else

	const slambench::values::PointCloudValue *latest_pc = static_cast<const slambench::values::PointCloudValue*>(output->GetValues().rbegin()->second);
	size_t size = latest_pc->GetPoints().size();

	auto colour = GetColour(outputs_colour_.at(output)->Get());

	glColor3f(std::get<0>(colour),std::get<1>(colour),std::get<2>(colour));
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(slambench::values::Point3DF) * size, latest_pc->GetPoints().data(), GL_STATIC_DRAW);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMultMatrixf((GLfloat*)latest_pc->GetTransform().data());

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glVertexPointer(3, GL_FLOAT, sizeof(slambench::values::Point3DF), 0);

	glEnableClientState(GL_VERTEX_ARRAY);

	glDrawArrays(GL_POINTS, 0, size);

	glDisableClientState(GL_VERTEX_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glPopMatrix();

	return true;
#endif

}


bool SLAMBenchUI_Pangolin::DrawColouredPointCloudOutput(slambench::outputs::BaseOutput* output)
{
	assert(output->GetType() == slambench::values::VT_COLOUREDPOINTCLOUD);
	output->SetKeepOnlyMostRecent(true);
	if(!output->IsActive() or output->GetValues().empty()) {
		return true;
	}

#ifdef HARRY_SPEEDUP

	auto timestamp = output->GetMostRecentValue().first;
	const slambench::values::ColoredPointCloudValue *latest_pc = static_cast<const slambench::values::ColoredPointCloudValue*>(output->GetMostRecentValue().second);
	size_t size = latest_pc->GetPoints().size();

	bool new_output = false;
	if(!pointcloud_state_.count(output)) {
		glCreateBuffers(1, &pointcloud_state_[output].VBO);
		new_output = true;
	}

	auto &state = pointcloud_state_[output];
	glBindBuffer(GL_ARRAY_BUFFER, state.VBO);
	if(new_output || state.timestamp != timestamp) {
		glBufferData(GL_ARRAY_BUFFER, sizeof(slambench::values::Point3DF) * size, latest_pc->GetPoints().data(), GL_STATIC_DRAW);
		state.timestamp = timestamp;
	}

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMultMatrixf((GLfloat*)latest_pc->GetTransform().data());

	glVertexPointer(3, GL_FLOAT, sizeof(slambench::values::ColoredPoint3DF), 0);
	glColorPointer(3, GL_UNSIGNED_BYTE, sizeof(slambench::values::ColoredPoint3DF), (void*)offsetof(slambench::values::ColoredPoint3DF, R));


	glEnableClientState(GL_VERTEX_ARRAY);
	glEnableClientState(GL_COLOR_ARRAY);

	glDrawArrays(GL_POINTS, 0, size);

	glDisableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glPopMatrix();

	return true;
#else

	const slambench::values::ColoredPointCloudValue *latest_pc = static_cast<const slambench::values::ColoredPointCloudValue*>(output->GetValues().rbegin()->second);
	size_t size = latest_pc->GetPoints().size();


	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(slambench::values::Point3DF) * size, latest_pc->GetPoints().data(), GL_STATIC_DRAW);

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMultMatrixf((GLfloat*)latest_pc->GetTransform().data());

	glBindBuffer(GL_ARRAY_BUFFER, vbo);
glVertexPointer(3, GL_FLOAT, sizeof(slambench::values::ColoredPoint3DF), 0);
	glColorPointer(3, GL_UNSIGNED_BYTE, sizeof(slambench::values::ColoredPoint3DF), (void*)offsetof(slambench::values::ColoredPoint3DF, R));

	glEnableClientState(GL_VERTEX_ARRAY);

	glDrawArrays(GL_POINTS, 0, size);

	glDisableClientState(GL_VERTEX_ARRAY);

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glPopMatrix();

	return true;
#endif

}

bool SLAMBenchUI_Pangolin::EnqueueFrame(slambench::values::FrameValue *frame) {
	frames_.push_back(*frame);
	return true;
}

bool SLAMBenchUI_Pangolin::DrawBackgrounds() {

	for (auto button : outputs_background_) {
		if (button.second->Get() and last_background_ != button.second) {
			//			std::cout << "New background" << std::endl;
			last_background_ = button.second;
			break;
		}

		if (!button.second->Get() and last_background_ == button.second) {
			//			std::cout << "unchecked background" << std::endl;
			last_background_ = nullptr;
			break;
		}
	}
	for (auto button : outputs_background_) {
		button.second->Ref().Set(last_background_ == button.second);
	}

	for(auto output_mgr : GetOutputManagers()) {
		std::lock_guard<FastLock> lock(output_mgr.second->GetLock());
		(void)lock; // I hate that werror so much

		for(auto output : *output_mgr.second) {
			if (output.second->IsActive() and (output.second->GetType() == slambench::values::VT_FRAME) and outputs_background_.at(output.second)->Get()) {
				this->drawBackground((slambench::values::FrameValue*)output.second->GetValues().rbegin()->second);
			}

		}
	}

	return true;
}

bool SLAMBenchUI_Pangolin::FollowPoses() {

	for (auto button : outputs_follow_) {
		if (button.second->Get() and last_follow_ != button.second) {
			//			std::cout << "New follow" << std::endl;
			last_follow_ = button.second;
			break;
		}

		if (!button.second->Get() and last_follow_ == button.second) {
			//			std::cout << "unchecked follow" << std::endl;
			last_follow_ = nullptr;
			break;
		}
	}

	bool should_follow = false;
	for (auto button : outputs_follow_) {
		if(last_follow_ == button.second) {
			should_follow = true;
		}
		button.second->Ref().Set(last_follow_ == button.second);
	}

	if(should_follow) {
		Eigen::Matrix4f followed_pose = Eigen::Matrix4f::Identity();
		bool got_pose = false;

		for(auto output_mgrp : GetOutputManagers()) {
			auto output_mgr = output_mgrp.second;
			std::lock_guard<FastLock> lock(output_mgr->GetLock());
			(void)lock; // I hate that werror so much

			for(auto output : *output_mgr) {

				if (output.second->GetType() != slambench::values::VT_POSE) {
					continue;
				}

				if(not output.second->IsActive()) {
					continue;
				}

				if(output.second->Empty()) {
					continue;
				}

				if (outputs_follow_.at(output.second)->Get()) {
					followed_pose = ((slambench::values::PoseValue*)(output.second->GetMostRecentValue().second))->GetValue();
					got_pose = true;
				}
			}
		}

		if(got_pose) {
			FollowPose(followed_pose);
		}
	}

	return true;
}


bool SLAMBenchUI_Pangolin::DrawQueuedFrames() {

	if (frames_.size() == 0) {
		return true;
	}

	Eigen::Vector4f winReg[MAX_WINDOW]; // (x1, y1, x2, y2)
	winReg[0] = {0.0f, 0.5f, 0.5f, 1.0f};
	winReg[1] = {0.5f, 0.5f, 1.0f, 1.0f};
	winReg[2] = {0.0f, 0.0f, 0.5f, 0.5f};
	winReg[3] = {0.5f, 0.0f, 1.0f, 0.5f};


	//display the image
	glColor3f(1.0,1.0,1.0);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glEnable(GL_TEXTURE_2D);



	for (unsigned int w = 0 ; w < frames_.size(); w++) {

		slambench::values::FrameValue &frame = frames_.at(w);

		glBindTexture(GL_TEXTURE_2D, textureId[w]);

        GLint format, type, internalformat;
        switch(frame.GetFormat()) {
            case slambench::io::pixelformat::G_I_8:
            case slambench::io::pixelformat::D_I_8:
                internalformat = GL_RGBA;
                format = GL_LUMINANCE;
                type   = GL_UNSIGNED_BYTE;
                break;
            case slambench::io::pixelformat::RGB_III_888:
                internalformat = GL_RGB;
                format = GL_RGB;
                type = GL_UNSIGNED_BYTE;
                break;
            case slambench::io::pixelformat::RGBA_IIII_8888:
                internalformat = GL_RGBA;
                format = GL_RGBA;
                type = GL_UNSIGNED_BYTE;
                break;
            case slambench::io::pixelformat::D_I_16:
                internalformat = GL_LUMINANCE;
                format = GL_LUMINANCE;
                type = GL_UNSIGNED_SHORT;
                break;
            default:
                // unknown type!
                abort();
        }

        glTexImage2D(GL_TEXTURE_2D, 0, internalformat, frame.GetWidth(), frame.GetHeight(), 0, format, type, frame.GetData());

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glBegin(GL_QUADS); {

			glTexCoord2f(0, 1);
			glVertex2f(winReg[w][0], winReg[w][1]);

			glTexCoord2f(1, 1);
			glVertex2f(winReg[w][2], winReg[w][1]);

			glTexCoord2f(1, 0);
			glVertex2f(winReg[w][2], winReg[w][3]);

			glTexCoord2f(0, 0);
			glVertex2f(winReg[w][0], winReg[w][3]);

		}
		glEnd();
	}


	glDisable(GL_TEXTURE_2D);

	frames_.clear();

	return true;
}



bool SLAMBenchUI_Pangolin::DrawFrameOutput(slambench::outputs::BaseOutput* output)
{
	if(!output->IsActive() or output->GetValues().empty()) {
		return true;
	}

	// get most recent frame output
	slambench::values::FrameValue *value = (slambench::values::FrameValue*)output->GetValues().rbegin()->second;
	EnqueueFrame(value);

	return true;
}

bool SLAMBenchUI_Pangolin::DrawFeature(slambench::values::FeatureValue* value)
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);
	glColor4f(1,1,1,1);

	auto tex_id = get_next_texture_id();
	glBindTexture(GL_TEXTURE_2D, tex_id);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, value->GetPatch().GetWidth(), value->GetPatch().GetHeight(), 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, value->GetPatch().GetData());
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	glBegin(GL_QUADS);

	const float uvs[] = {0, 0, 0, 1, 1, 1, 1, 0};
	const float vertices[] = {-0.05, -0.05, 0, 0.05, -0.05, 0, 0.05, 0.05, 0, -0.05, 0.05, 0};
	for(int i = 0; i < 4; ++i) {
		float vertex[] = {vertices[3*i], vertices[1+3*i], vertices[2+3*i]};
		vertex[0] += value->GetPose()(0,1);
		vertex[1] += value->GetPose()(0,2);
		vertex[2] += value->GetPose()(0,3);
		glTexCoord2fv(uvs + (2*i));
		glVertex3fv(vertex);
	}

	glEnd();

	glDisable(GL_TEXTURE_2D);
	glDisable(GL_DEPTH_TEST);

	return true;
}

bool SLAMBenchUI_Pangolin::DrawList(slambench::outputs::BaseOutput* output)
{

	if (output->Empty()) {
		return true;
	}
	// take the most recent list
	auto list_value = (slambench::values::ValueListValue*)output->GetMostRecentValue().second;
	auto &list = list_value->GetValue();

	for(auto i : list) {
		switch(i->GetType()) {
		case slambench::values::VT_FEATURE:
			DrawFeature(static_cast<slambench::values::FeatureValue*>(i));
			break;
		default:
			throw std::logic_error("Unknown list type");
		}
	}
	return true;
}

bool SLAMBenchUI_Pangolin::DrawString(slambench::outputs::BaseOutput* value)
{

	if (value->Empty()) {
		return true;
	}

	auto string_value = (slambench::values::TypeForVT<slambench::values::VT_STRING>::type*)value->GetMostRecentValue().second;
	outputs_text_.at(value)->Ref().Set(string_value->GetValue());

	return true;
}


bool SLAMBenchUI_Pangolin::DrawOutput(slambench::outputs::BaseOutput* output)
{
	//	std::cout << "Drawing an output " << output->GetType() << std::endl;
	if(outputs_visible_.count(output) != 0 && !outputs_visible_.at(output)->Get()) {
		return true;
	}
	if(!output->IsActive()) {
		return true;
	}

	switch(output->GetType()) {
	case slambench::values::VT_POSE:
		return DrawPoseOutput(output);
	case slambench::values::VT_COLOUREDPOINTCLOUD:
		return DrawColouredPointCloudOutput(output);
	case slambench::values::VT_POINTCLOUD:
		return DrawPointCloudOutput(output);
	case slambench::values::VT_FRAME:
		return DrawFrameOutput(output);
	case slambench::values::VT_LIST:
		return DrawList(output);
	case slambench::values::VT_STRING:
		return DrawString(output);
	default:
		return false;
	}
}

void SLAMBenchUI_Pangolin::lockOutputs()
{
	for(auto output_mgr : GetOutputManagers()) {
		output_mgr.second->GetLock().lock();
	}
}

void SLAMBenchUI_Pangolin::unlockOutputs()
{
	for(auto output_mgr : GetOutputManagers()) {
		output_mgr.second->GetLock().unlock();
	}
}



bool SLAMBenchUI_Pangolin::process(){

	//		std::cout << "Processing!" << std::endl;
	/***
	 * Running system
	 */

	if(CanFreeRun()) {
		if(freeRunning->Get() != isFreeRunning) {
			isFreeRunning = freeRunning->Get();
			std::unique_lock<std::mutex> lck(step_mutex);
			step_cv.notify_all();
		}
	}

	if(CanStep()) {
		if(nextFrame->Ref().Get()) {
			nextFrame->Ref().Set(false);
			std::unique_lock<std::mutex> lck(step_mutex);
			step_cv.notify_all();
		}
	}


	for(auto output : outputs_enabled_) {
		bool should_be_active = output.second->Get();
		if(should_be_active != output.first->IsActive()) {
			lockOutputs();
			output.first->SetActive(should_be_active);
			unlockOutputs();
		}
	}

	{
		// Update aspect ratio of display to match that of the window

		auto &display = pangolin::Display("cam");
		double aspect = (double)(pangolin::DisplayBase().v.w-360) / pangolin::DisplayBase().v.h;
		display.SetAspect(aspect);
	}


	// Move to BIG screen
	// ********************

	pangolin::Display("cam").Activate(s_cam);


	// Draw blue background
	// ********************
	glClearColor(0.05, 0.05, 0.3, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	FollowPoses();

	DrawBackgrounds();

	for(auto output_mgr : GetOutputManagers()) {
		std::lock_guard<FastLock> lock(output_mgr.second->GetLock());
		(void)lock; // I hate that werror so much

		for(auto output : *output_mgr.second) {
			DrawOutput(output.second);
		}
	}

	DrawQueuedFrames();

	DrawAxis();

	this->postCall();

	return true;

};

void SLAMBenchUI_Pangolin::DrawAxis() {


	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();


	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	auto mat = s_cam.GetModelViewMatrix();
	mat.m[12] = -0.85;
	mat.m[13] = 0.85;
	mat.m[14] = -0.5;
	mat.m[15] = 1;

	mat.Multiply();

	glBegin(GL_LINES);

	glColor3f(1,0.5,0.5);
	glVertex3f(0, 0, 0);
	glVertex3f(0.1, 0, 0);

	glColor3f(0.5,1,0.5);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0.1, 0);

	glColor3f(0.5,0.5,1);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 0.1);

	glEnd();

	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
}

unsigned int SLAMBenchUI_Pangolin::get_next_texture_id() {
	if(next_feature_texture_id_idx == feature_texture_ids.size()) {
		uint texture_id;
		glGenTextures(1, &texture_id);
		feature_texture_ids.push_back(texture_id);
	}

	return feature_texture_ids.at(next_feature_texture_id_idx++);
}




void SLAMBenchUI_Pangolin::drawBackground(slambench::values::FrameValue * frame) {

	glBindTexture(GL_TEXTURE_2D, textureId[0]);

	glColor3f(1.0,1.0,1.0);
	glEnable(GL_TEXTURE_2D);
	glDisable(GL_DEPTH_TEST);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	glOrtho( 0, 1, 0, 1, -1, 1 );


	GLint format, type, internalformat;
	switch(frame->GetFormat()) {
		case slambench::io::pixelformat::G_I_8:
		case slambench::io::pixelformat::D_I_8:
            internalformat = GL_RGBA;
			format = GL_LUMINANCE;
			type   = GL_UNSIGNED_BYTE;
			break;
		case slambench::io::pixelformat::RGB_III_888:
            internalformat = GL_RGB;
			format = GL_RGB;
			type = GL_UNSIGNED_BYTE;
			break;
		case slambench::io::pixelformat::RGBA_IIII_8888:
            internalformat = GL_RGBA;
			format = GL_RGBA;
			type = GL_UNSIGNED_BYTE;
			break;
        case slambench::io::pixelformat::D_I_16:
            internalformat = GL_LUMINANCE;
            format = GL_LUMINANCE;
            type = GL_UNSIGNED_SHORT;
            break;
        default:
			// unknown type!
			abort();
		}

	glTexImage2D(GL_TEXTURE_2D, 0, internalformat, frame->GetWidth(), frame->GetHeight(), 0, format, type, frame->GetData());

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glBegin( GL_QUADS ); {

		glTexCoord2f(0,1);
		glVertex2f(0.0f, 0.0f);

		glTexCoord2f(1,1);
		glVertex2f(1, 0.0f);

		glTexCoord2f(1,0);
		glVertex2f(1, 1);

		glTexCoord2f(0,0);
		glVertex2f(0.0f,  1);
	}
	glEnd();




	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_TEXTURE_2D);




}

void SLAMBenchUI_Pangolin::drawTrajectory(const slambench::outputs::TrajectoryInterface *trajectory, float R , float G, float B , int print_poses) {



	glPushMatrix();

	glColor3f(R, G, B);

	glBegin(GL_LINES);

	bool first = true;
	auto traj = trajectory->GetAll();

	if(traj.empty()) {
		return;
	}

	for(auto &point :  traj) {
		const auto &mat = point.second.GetValue();
		if (first) {
			glVertex3f(mat(0,3),mat(1,3),mat(2,3));
			first = false;
		}
		glVertex3f(mat(0,3),mat(1,3),mat(2,3));
		glVertex3f(mat(0,3),mat(1,3),mat(2,3));
	}

	glEnd();
	glPopMatrix();



	int print_me = 0;
	int frequency =  print_poses  ;
	//std::cout << "frequency =" << frequency <<std::endl;
	for(auto &point :  traj) {
		print_me++;
		if (frequency != 0 and print_me > frequency) {print_me = 0;}
		else {continue;}

		Eigen::Matrix4f matfrtu =   point.second.GetValue();

		matfrtu.block<3,1>(0,0) = matfrtu.block<3,1>(0,0) / (matfrtu.block<3,1>(0,0)).norm();
		matfrtu.block<3,1>(0,1) = matfrtu.block<3,1>(0,1) / (matfrtu.block<3,1>(0,1)).norm();
		matfrtu.block<3,1>(0,2) = matfrtu.block<3,1>(0,2) / (matfrtu.block<3,1>(0,2)).norm();

		this->drawFrustum( matfrtu, R,G,B);
	}

	Eigen::Matrix4f matfrtu =   traj.rbegin()->second.GetValue();

	matfrtu.block<3,1>(0,0) = matfrtu.block<3,1>(0,0) / (matfrtu.block<3,1>(0,0)).norm();
	matfrtu.block<3,1>(0,1) = matfrtu.block<3,1>(0,1) / (matfrtu.block<3,1>(0,1)).norm();
	matfrtu.block<3,1>(0,2) = matfrtu.block<3,1>(0,2) / (matfrtu.block<3,1>(0,2)).norm();

	this->drawFrustum( matfrtu, R,G,B);


}

void SLAMBenchUI_Pangolin::FollowPose(Eigen::Matrix4f currPose ) {

	pangolin::OpenGlMatrix mv;

	Eigen::Matrix3f currRot = currPose.topLeftCorner(3, 3);

	Eigen::Quaternionf currQuat(currRot);
	Eigen::Vector3f forwardVector(0, 0, 1);
	Eigen::Vector3f upVector(0, -1, 0); // Flip cloud ?

	Eigen::Vector3f forward = (currQuat * forwardVector).normalized();
	Eigen::Vector3f up = (currQuat * upVector).normalized();

	Eigen::Vector3f eye(currPose(0, 3), currPose(1, 3), currPose(2, 3));



	Eigen::Vector3f at = eye + forward;

	Eigen::Vector3f z = (eye - at).normalized();  // Forward
	Eigen::Vector3f x = up.cross(z).normalized(); // Right
	Eigen::Vector3f y = z.cross(x);

	Eigen::Matrix4d m;
	m << x(0),  x(1),  x(2),  -(x.dot(eye)),
			y(0),  y(1),  y(2),  -(y.dot(eye)),
			z(0),  z(1),  z(2),  -(z.dot(eye)),
			0,     0,     0,              1;

	memcpy(&mv.m[0], m.data(), sizeof(Eigen::Matrix4d));

	s_cam.SetModelViewMatrix(mv);



}

void SLAMBenchUI_Pangolin::drawFrustum(Eigen::Matrix4f currPose, float R , float G, float B) {

	glPushMatrix();

	glMultMatrixf((GLfloat*) currPose.data());
	glColor3f(R, G, B);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0.05 * (0 - cx) / fx, 0.05 * (0 - cy) / fy, 0.05);
	glVertex3f(0, 0, 0);
	glVertex3f(0.05 * (0 - cx) / fx, 0.05 * (height - 1 - cy) / fy, 0.05);
	glVertex3f(0, 0, 0);
	glVertex3f(0.05 * (width - 1 - cx) / fx, 0.05 * (height - 1 - cy) / fy, 0.05);
	glVertex3f(0, 0, 0);
	glVertex3f(0.05 * (width - 1 - cx) / fx, 0.05 * (0 - cy) / fy, 0.05);
	glVertex3f(0.05 * (width - 1 - cx) / fx, 0.05 * (0 - cy) / fy, 0.05);
	glVertex3f(0.05 * (width - 1 - cx) / fx, 0.05 * (height - 1 - cy) / fy, 0.05);
	glVertex3f(0.05 * (width - 1 - cx) / fx, 0.05 * (height - 1 - cy) / fy, 0.05);
	glVertex3f(0.05 * (0 - cx) / fx, 0.05 * (height - 1 - cy) / fy, 0.05);
	glVertex3f(0.05 * (0 - cx) / fx, 0.05 * (height - 1 - cy) / fy, 0.05);
	glVertex3f(0.05 * (0 - cx) / fx, 0.05 * (0 - cy) / fy, 0.05);
	glVertex3f(0.05 * (0 - cx) / fx, 0.05 * (0 - cy) / fy, 0.05);
	glVertex3f(0.05 * (width - 1 - cx) / fx, 0.05 * (0 - cy) / fy, 0.05);
	glEnd();
	glBegin(GL_LINES);

	glVertex3f(0,0,0);
	glVertex3f(0,-0.05,0);

	glVertex3f(0.02,-0.03,0);
	glVertex3f(0,   -0.05,0);

	glVertex3f(-0.02,-0.03,0);
	glVertex3f(0,    -0.05,0);

	glEnd();
	glPopMatrix();

}


void SLAMBenchUI_Pangolin::postCall() {


	struct sysinfo memInfo;


	sysinfo (&memInfo);
	long long totalVirtualMem = memInfo.totalram;
	//Add other values in next statement to avoid int overflow on right hand side...
	totalVirtualMem += memInfo.totalswap;
	totalVirtualMem *= memInfo.mem_unit;

	long long virtualMemUsed = memInfo.totalram - memInfo.freeram;
	//Add other values in next statement to avoid int overflow on right hand side...
	virtualMemUsed += memInfo.totalswap - memInfo.freeswap;
	virtualMemUsed *= memInfo.mem_unit;

	long long totalPhysMem = memInfo.totalram;
	//Multiply in next statement to avoid int overflow on right hand side...
	totalPhysMem *= memInfo.mem_unit;

	long long physMemUsed = memInfo.totalram - memInfo.freeram;
	//Multiply in next statement to avoid int overflow on right hand side...
	physMemUsed *= memInfo.mem_unit;


	VRSS->operator=(getValueVmRSS()/ 1000);
	VSIZE->operator=(getValueVmSize()/ 1000);

	TVM->operator=(totalVirtualMem / 1000000);
	CVM->operator=(virtualMemUsed / 1000000 );
	TPM->operator=(totalPhysMem / 1000000 );
	CPM->operator=(physMemUsed / 1000000 );
	frameCount->operator=(this->frame);

	pangolin::FinishFrame();

	glFinish();
}

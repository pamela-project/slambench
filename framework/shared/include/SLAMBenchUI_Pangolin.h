
#ifndef SLAMBENCHUI_PANGOLIN_H_
#define SLAMBENCHUI_PANGOLIN_H_

#define GLM_FORCE_RADIANS
#include <pangolin/pangolin.h>
#include <map>
#include <sysutils.h>
#include <SLAMBenchUI.h>
#include <SLAMBenchAPI.h>

#include <condition_variable>
#include <mutex>

#include <outputs/TrajectoryInterface.h>

#define MAX_WINDOW 4



//#define GEN_FOLLOW_BUTTON_NAME(context_name,item) (item.name != "")  ? "ui.Follow " + ((context_name!="") ?context_name + " ":"") + item.name :  "ui.Follow " + ((context_name!="")? context_name + " ":"") + " " + item.type ;
#define GEN_FOLLOW_BUTTON_NAME(context_name,item) (item.name != "")  ? "ui.Follow." + ((context_name!="") ?context_name + " ":"") + item.name :  "ui.Follow." + context_name + " " + item.type ;

//#define GEN_SHOW_BOX_NAME(context_name,item) (item.first.name != "")  ? "ui.Show " + context_name + " " + item.first.name :  "ui.Show " + context_name + " " + item.first.type ;
#define GEN_SHOW_BOX_NAME(context_name,item) (item.first.name != "")  ? "ui." + context_name + ".Show " + item.first.name :  "ui." + context_name + ".Show " + item.first.type ;

class PoseOutputDrawContext;

class SLAMBenchUI_Pangolin : public SLAMBenchUI
{

    public:
        SLAMBenchUI_Pangolin();
		
		void InitialiseOutputs();
		void AddControlsForOutput(const std::string &prefix, slambench::outputs::BaseOutput *output);
		void AddBackgroundControls();
		void AddFollowControls();
		
		bool DrawPoseOutput(slambench::outputs::BaseOutput *output);
		bool DrawPointCloudOutput(slambench::outputs::BaseOutput *output);
		bool DrawColouredPointCloudOutput(slambench::outputs::BaseOutput *output);
		bool DrawFrameOutput(slambench::outputs::BaseOutput *output);
		bool DrawOutput(slambench::outputs::BaseOutput *output);
		bool DrawList(slambench::outputs::BaseOutput *output);
		bool DrawFeature(slambench::values::FeatureValue* value);
		bool DrawString(slambench::outputs::BaseOutput* value);
        void drawTrajectory(const slambench::outputs::TrajectoryInterface *traj, float R, float G, float B, int print_poses = 0 );
        void drawFrustum(Eigen::Matrix4f currPose, float R, float G, float B);
		
        bool process();
        void preCall();
		void DrawAxis();
        void drawBackground(slambench::values::FrameValue * frame);


		unsigned int get_next_texture_id();
		
		void FollowPose(Eigen::Matrix4f currPose ) ;
        void postCall();
        void StopTracking();
        ~SLAMBenchUI_Pangolin();

		bool CanFreeRun() override;
		bool CanStep() override;
		bool ClearFreeRunning() override;
		bool IsFreeRunning() override;
		bool SetFreeRunning() override;
		bool WaitForFrame() override;

    private:
		std::map<slambench::outputs::BaseOutput *, pangolin::Var<bool>*> outputs_enabled_, outputs_visible_, outputs_follow_, outputs_background_;
		std::map<slambench::outputs::BaseOutput *, PoseOutputDrawContext*> pose_output_draw_contexts_;
		
		std::map<slambench::outputs::BaseOutput *, pangolin::Var<std::string>*> outputs_text_;
		
		pangolin::Var<bool>* last_background_ = nullptr;
		pangolin::Var<bool>* last_follow_ = nullptr;

		std::map<slambench::outputs::BaseOutput *, pangolin::Var<double>*> outputs_colour_;
		
		typedef struct {
			slambench::TimeStamp timestamp;
			GLuint VBO;
		} PointCloudState;
		std::map<slambench::outputs::BaseOutput *, PointCloudState> pointcloud_state_;
		
		std::vector<slambench::values::FrameValue> frames_;
		bool EnqueueFrame(slambench::values::FrameValue*);
		bool DrawQueuedFrames();
		bool DrawBackgrounds();
		bool FollowPoses();
		void lockOutputs();
		void unlockOutputs();

        pangolin::Var<long long> * frameCount;
        pangolin::Var<long long> * VRSS;
        pangolin::Var<long long> * VSIZE;
        pangolin::Var<long long> * TVM;
        pangolin::Var<long long> * CVM;
        pangolin::Var<long long> * TPM;
        pangolin::Var<long long> * CPM;
        pangolin::Var<bool> * showBackground;
        pangolin::Var<bool> * followPose;

        pangolin::Var<int> * showEveryPose;
		pangolin::Var<bool> * freeRunning;
        pangolin::Var<bool> * nextFrame;
        pangolin::Var<bool> * unFollow;

        // RGB View
        pangolin::OpenGlRenderState s_cam;
        GLuint vbo;

		std::vector<uint> feature_texture_ids;
		uint32_t next_feature_texture_id_idx;
		
        uint textureId[MAX_WINDOW];
		std::condition_variable step_cv;
		std::mutex step_mutex;
		bool isFreeRunning;
};


#endif /* SLAMBENCHUI_PANGOLIN_H_ */

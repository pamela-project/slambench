/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#include "metrics/ATEMetric.h"
#include <outputs/TrajectoryInterface.h>
#include "outputs/TrajectoryAlignmentMethod.h"
#include <boost/optional.hpp>

using namespace slambench::metrics;



ATEMetric::ATEMetric(const slambench::outputs::TrajectoryInterface* tested_trajectory, const slambench::outputs::TrajectoryInterface * ground_truth) : Metric("ATE"), trajectory_(tested_trajectory), ground_truth_(ground_truth)
{
	next_gt_ts_ = {0,0};
}

const slambench::values::ValueDescription &ATEMetric::GetValueDescription() const {
	static const slambench::values::ValueDescription desc = slambench::values::ValueDescription({
		std::make_pair("AbsoluteError",  slambench::values::VT_DOUBLE),
        std::make_pair("OrientationError",  slambench::values::VT_DOUBLE),
        std::make_pair("MeanATE",  slambench::values::VT_DOUBLE),
        std::make_pair("MaxATE",  slambench::values::VT_DOUBLE)});

	return  desc;
}

const std::string& ATEMetric::GetDescription() const
{
	static std::string desc = "Absolute Trajectory Error";
	return desc;
}

void ATEMetric::MeasureStart(Phase* phase)
{
	(void)phase;
}

void ATEMetric::MeasureEnd(Phase* phase)
{
	(void)phase;
	latest_trajectory_ = trajectory_->GetAll();
}

Value *ATEMetric::GetValue(Phase* phase)
{
	(void)phase;

	auto gt_traj = ground_truth_->GetAll();
	auto es_traj = latest_trajectory_;

	if(gt_traj.size() == 0) {
			std::cerr << "**** Error: Empty GT." << std::endl;
			return new values::TypedValue<double>((double(std::nan(""))));
	}

	auto gt_iterator = gt_traj.begin() ;
	auto es_iterator = es_traj.begin() ;

	double LastATE   = 0.0;
	double MaxATE   = 0.0;
	double SumATE   = 0.0;
	double CountATE = 0.0;
	double LastAOE   = 0.0;


	for (es_iterator = es_traj.begin() ; es_iterator != es_traj.end() ; es_iterator ++) {



		// Find closest gt point
		//****************************************************************************************


		// Step 1 : ensure GT in the futre exists
		//***************************************

		while((gt_iterator->first < es_iterator->first) && gt_iterator != gt_traj.end() ) {
			gt_iterator++;
		}


		if(gt_iterator == gt_traj.end()) {
			std::cerr << "**** ATE Error: No more groundtruth to compare with." << std::endl;
			LastATE = double(std::nan(""));
			break;
		}


		// Step 2 : Closest Future
		//************************

		auto closest = gt_iterator;
		for (; gt_iterator != gt_traj.end(); ++gt_iterator) {
			auto GT_TS  = gt_iterator;
			if ( GT_TS->first > es_iterator->first ) {
				break;
			}

			auto CL_TS  = gt_iterator;
			if (es_iterator->first - CL_TS->first > es_iterator->first - GT_TS->first) {
				closest = GT_TS;
			}
		}
		gt_iterator = closest;


		// compute Absolute Error for this point
		//****************************************************************************************

		const Eigen::Matrix4f& gt_pose      = gt_iterator->second.GetValue();
		const Eigen::Matrix4f& aligned_pose = es_iterator->second.GetValue();

		Eigen::Vector3f diff = {
					std::abs(gt_pose(0,3) - aligned_pose(0,3))  ,
					std::abs(gt_pose(1,3) - aligned_pose(1,3))  ,
					std::abs(gt_pose(2,3) - aligned_pose(2,3)) } ;

		LastATE = std::sqrt( diff[0] * diff[0] +  diff[1] * diff[1] +  diff[2] * diff[2]);


		// compute Orientation Error for this point
		//****************************************************************************************

		Eigen::Matrix3f gt_rot = gt_pose.block<3,3>(0,0);
		Eigen::Matrix3f aligned_rot = aligned_pose.block<3,3>(0,0);
		// the rotation matrix in aligned pose has a scale factor, need be de-scaled to be a valid rotation matrix
		aligned_rot /= aligned_rot.block<3,1>(0,0).norm();
		Eigen::AngleAxisf diff_angle(gt_rot.transpose() * aligned_rot);
		LastAOE = diff_angle.angle() / M_PI * 180.;

		// cumul values
		//****************************************************************************************

		MaxATE  = std::max(MaxATE,LastATE);
		SumATE += LastATE ;
		CountATE++;

	}


	double MeanATE = SumATE / CountATE;

	auto absolute_error = new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(LastATE);
	auto orientation_error = new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(LastAOE);
	auto mean_ate = new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(MeanATE);
	auto max_ate  = new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(MaxATE);

	return new slambench::values::TypeForVT<slambench::values::VT_COLLECTION>::type({
				{"AbsoluteError",absolute_error},
				{"OrientationError",orientation_error},
				{"MeanATE",mean_ate},
				{"MaxATE",max_ate},
			});


}

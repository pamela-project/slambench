/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "metrics/RPEMetric.h"
#include "outputs/TrajectoryAlignmentMethod.h"
#include <boost/optional.hpp>

using namespace slambench::metrics;



RPEMetric::RPEMetric(const slambench::outputs::TrajectoryInterface* tested_trajectory, const slambench::outputs::TrajectoryInterface * ground_truth) : Metric("RPE"), trajectory_(tested_trajectory), ground_truth_(ground_truth)
{
	next_gt_ts_ = {0,0};
}

const slambench::values::ValueDescription &RPEMetric::GetValueDescription() const {
	static const slambench::values::ValueDescription desc = slambench::values::ValueDescription({
		{"RPE_RMSE",  slambench::values::VT_DOUBLE}});

	return  desc;
}

const std::string& RPEMetric::GetDescription() const
{
	static std::string desc = "Average Trajectory Error";
	return desc;
}

void RPEMetric::MeasureStart(Phase* phase)
{
	(void)phase;
}


void RPEMetric::MeasureEnd(Phase* phase)
{
	(void)phase;
	latest_trajectory_ = trajectory_->GetAll();
}

Value *RPEMetric::GetValue(Phase* phase)
{
	(void)phase;

	(void)phase;

	auto gt_traj = ground_truth_->GetAll();
	auto es_traj = latest_trajectory_;

	if(gt_traj.size() == 0) {
			std::cerr << "**** Error: Empty GT." << std::endl;
			return new values::TypedValue<double>((double(std::nan(""))));
	}

	double norms = 0.0;
	int count    = 0;

    // TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
	// Previously there was i_delta in original implementation
	// I drop the feature for now, will put it back later
	// TODO: hHe solution here is to have ParameterComponent for Metrics, so this parameter is visible externally

	// in RPE we compare shift from previous point, we need to store them
	auto gt_previous_iterator = gt_traj.end() ;
	auto es_previous_iterator = es_traj.end() ;

	auto gt_iterator = gt_traj.begin() ;
	auto es_iterator = es_traj.begin() ;

	bool first_point = true;

	for (es_iterator = es_traj.begin() ; es_iterator != es_traj.end() ; es_iterator ++) {


		// Find closest gt point
		//****************************************************************************************


		// Step 1 : ensure GT in the futre exists
		//***************************************

		while(gt_iterator->first < es_iterator->first ) {
			gt_iterator++;
		}

		if(gt_iterator == gt_traj.end()) {
			std::cerr << "**** Error: No more groundtruth to compare with." << std::endl;
			return new values::TypedValue<double>((double(std::nan(""))));
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


		// If at least two points are processed, we can compute  Relative Pose Error
		//****************************************************************************************

		if (!first_point) {

	        auto pose_gt_i    = gt_previous_iterator->second.GetValue();
	        auto pose_gt_j    = gt_iterator->second.GetValue();
	        auto pose_gt_diff = pose_gt_i * pose_gt_j.inverse();

	        auto pose_es_i    = es_previous_iterator->second.GetValue();
	        auto pose_es_j    = es_iterator->second.GetValue();
	        auto pose_es_diff = pose_es_i * pose_es_j.inverse();



	        Eigen::Matrix4f E = pose_es_diff * pose_gt_diff.inverse();
	        Eigen::Vector3f v = E.block<3,1>(0,3);
	        double length = v.norm();
	        norms = norms + length*length;
	        count++;

		}


		// store previous values
		//****************************************************************************************
		gt_previous_iterator = gt_iterator  ;
		es_previous_iterator = es_iterator  ;
		first_point = false;

	}


	double RPE = (count == 0) ? 0 : std::sqrt(norms/ (double)count);


	auto rpe_rmse = new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(RPE);


	return new slambench::values::TypeForVT<slambench::values::VT_COLLECTION>::type({
				{"RPE_RMSE",rpe_rmse},
			});

}

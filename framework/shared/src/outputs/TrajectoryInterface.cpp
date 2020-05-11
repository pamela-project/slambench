/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "outputs/TrajectoryInterface.h"

#include <iostream>

using namespace slambench;
using namespace slambench::outputs;
using namespace slambench::values;

PoseOutputTrajectoryInterface::PoseOutputTrajectoryInterface(BaseOutput* pose_output) : pose_output_(pose_output), newest_point_(TimeStamp::get(0,0))
{
	assert(pose_output_->GetType() == VT_POSE);
}

PoseOutputTrajectoryInterface::~PoseOutputTrajectoryInterface()
{

}

PoseValue PoseOutputTrajectoryInterface::Get(const TimeStamp& when) const
{
	return *static_cast<const PoseValue*>(pose_output_->GetValues().at(when));
}

TrajectoryValue::pose_container_t PoseOutputTrajectoryInterface::GetAll() const
{

	if((       ((!pose_output_->IsActive())  ||  !pose_output_->Empty())
			&&  ((!pose_output_->IsActive())  ||  pose_output_->GetMostRecentValue().first > newest_point_))
			|| cached_traj_.empty()) {
		recalculate();
	}
	
	return cached_traj_;
}
void PoseOutputTrajectoryInterface::recalculate() const {
	cached_traj_.clear();
	
	for(auto i : pose_output_->GetValues()) {
		const PoseValue *point = static_cast<const PoseValue*>(i.second);
		cached_traj_.insert({i.first, *point});
	}
	
	if(!cached_traj_.empty()) {
		newest_point_ = cached_traj_.rbegin()->first;
	}
}


TrajectoryValueWrapper::TrajectoryValueWrapper(const TrajectoryValue *t) : trajectoryValue(t) {
}

values::PoseValue TrajectoryValueWrapper::Get(const TimeStamp &when) const {
	return trajectoryValue->GetPoints().at(when);
}

values::TrajectoryValue::pose_container_t TrajectoryValueWrapper::GetAll() const {
	return trajectoryValue->GetPoints();
}

TrajectoryOutputInterface::TrajectoryOutputInterface(const BaseOutput *t):
	trajectory_output(t)
{ }

values::PoseValue TrajectoryOutputInterface::Get(const TimeStamp &when) const {
    const values::Value *raw_value = trajectory_output->GetMostRecentValue().second;
    const values::TrajectoryValue *tv = reinterpret_cast<const values::TrajectoryValue*>(raw_value);

    return tv->GetPoints().at(when);
}

values::TrajectoryValue::pose_container_t TrajectoryOutputInterface::GetAll() const {

    const values::Value *raw_value = trajectory_output->GetMostRecentValue().second;
    const values::TrajectoryValue *tv = reinterpret_cast<const values::TrajectoryValue*>(raw_value);

    return tv->GetPoints();
}




/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include "outputs/Output.h"
#include "values/Value.h"
#include "outputs/TrajectoryAlignmentMethod.h"
#include "outputs/TrajectoryInterface.h"

#include <pcl/registration/icp.h>
#include <pcl/io/ply_io.h>

#include <iostream>

using namespace slambench::outputs;
using slambench::values::PointCloudValue;
typedef pcl::PointXYZ point_t;

BaseOutput::BaseOutput(const std::string& name, const values::ValueDescription &type, bool main_output) : name_(name), type_(type), only_keep_most_recent_(false), active_(false), main_(main_output)
{

}

const BaseOutput::value_map_t::value_type& BaseOutput::GetMostRecentValue() const
{
	if(GetValues().empty()) {
		throw std::logic_error("No values in output");
	}

	return *GetValues().rbegin();
}

bool BaseOutput::Empty() const
{
	return GetValues().empty();
}


Output::Output(const std::string& name, values::ValueType type, bool main_output) : BaseOutput(name, type, main_output)
{

}

void Output::AddPoint(timestamp_t time, const values::Value *value)
{
	assert(value->GetType() == GetType());
	assert(IsActive());
	if(GetKeepOnlyMostRecent()) {
		for(auto i : values_) {
			delete i.second;
		}
		values_.clear();
	}
	values_[time] = value;

	Updated();
}

const Output::value_map_t& Output::GetValues() const
{
	assert(IsActive());
	return values_;
}

const BaseOutput::value_map_t::value_type& Output::GetMostRecentValue() const
{
	if(values_.empty()) {
		throw std::logic_error("No values available in output");
	}
	return *values_.rbegin();
}

void BaseOutput::Updated() const
{
	for(auto i : update_callbacks_) {
		i(this);
	}
}

DerivedOutput::DerivedOutput(const std::string &name, values::ValueType type, const std::initializer_list<BaseOutput*> &derived_from, bool main) : BaseOutput(name, type, main), derived_from_(derived_from), up_to_date_(false)
{
	for(BaseOutput *i : derived_from_) {
		i->AddUpdateCallback([this](const BaseOutput*){this->Invalidate();});
	}
}

bool DerivedOutput::Empty() const
{
	if(!up_to_date_) {
		recalculate();
	}

	return cached_values_.empty();
}

void DerivedOutput::recalculate() const
{
	const_cast<DerivedOutput*>(this)->Recalculate();
	const_cast<DerivedOutput*>(this)->up_to_date_ = true;
	Updated();
}


const BaseOutput::value_map_t::value_type& DerivedOutput::GetMostRecentValue() const
{
	if(!up_to_date_) {
		recalculate();
	}

	return *cached_values_.rbegin();
}

const BaseOutput::value_map_t& DerivedOutput::GetValues() const
{
	if(!up_to_date_) {
		recalculate();
	}

	return cached_values_;
}

void DerivedOutput::Invalidate()
{
	up_to_date_ = false;
}

BaseOutput::value_map_t& DerivedOutput::GetCachedValueMap()
{
	return cached_values_;
}


AlignmentOutput::AlignmentOutput(const std::string& name, TrajectoryInterface* gt_trajectory, BaseOutput* trajectory, TrajectoryAlignmentMethod *method) : DerivedOutput(name, values::VT_MATRIX, {trajectory}), freeze_(false), gt_trajectory_(gt_trajectory), trajectory_(trajectory), method_(method)
{

}

void AlignmentOutput::Recalculate()
{
	if (freeze_) return;
	assert(GetKeepOnlyMostRecent());
	auto &target = GetCachedValueMap();

	for(auto i : target) {
		delete i.second;
	}
	target.clear();

	if(trajectory_->Empty()) {
		return;
	}

	slambench::outputs::PoseOutputTrajectoryInterface traj_int(trajectory_);

    //transformation_ = (*method_)(gt_trajectory_->GetAll(), traj_int.GetAll());
    auto transformation = (*method_)(gt_trajectory_->GetAll(), traj_int.GetAll());
	auto &last_point = trajectory_->GetMostRecentValue();

	target.insert({last_point.first, new values::TypedValue<Eigen::Matrix4f>(transformation)});
}

AlignedPoseOutput::AlignedPoseOutput(const std::string& name, AlignmentOutput* alignment, BaseOutput* pose_output) : DerivedOutput(name, values::VT_POSE, {alignment, pose_output}), alignment_(alignment), pose_output_(pose_output)
{

}

void AlignedPoseOutput::Recalculate()
{
	auto &target = GetCachedValueMap();

	for(auto i : target) {
		delete i.second;
	}
	target.clear();

	if(!alignment_->IsActive()) return;
	if(alignment_->Empty()) return;
	if(!pose_output_->IsActive()) return;
	if(pose_output_->Empty()) return;

    auto newest_alignment = alignment_->GetMostRecentValue().second;
    values::TypedValue<Eigen::Matrix4f> *mv = (values::TypedValue<Eigen::Matrix4f>*)newest_alignment;

    for(auto traj_point : pose_output_->GetValues()) {
        auto pose = (values::PoseValue*)traj_point.second;

		Eigen::Matrix4f tmp = mv->GetValue() * pose->GetValue();
		target.insert({traj_point.first, const_cast<values::PoseValue*>(new values::PoseValue(tmp))});
	}
}

AlignedPointCloudOutput::AlignedPointCloudOutput(const std::string& name, AlignmentOutput* alignment, BaseOutput* pc_output) : DerivedOutput(name, values::VT_POINTCLOUD, {alignment, pc_output}), alignment_(alignment), pointcloud_(pc_output)
{
	SetKeepOnlyMostRecent(true);
}

void AlignedPointCloudOutput::Recalculate()
{
	assert(GetKeepOnlyMostRecent());
	auto &target = GetCachedValueMap();

	for(auto i : target) {
		delete i.second;
	}
	target.clear();

	if(!pointcloud_->IsActive() || pointcloud_->GetValues().empty() || !alignment_->IsActive() || alignment_->GetValues().empty()) {
		return;
	}

	auto latest_point = pointcloud_->GetMostRecentValue();
	auto latest_pc = (values::PointCloudValue*)latest_point.second;
	auto latest_ts = latest_point.first;

	auto latest_alignment = ((values::TypedValue<Eigen::Matrix4f>*)alignment_->GetMostRecentValue().second)->GetValue();

	auto new_pc = new values::PointCloudValue(*latest_pc);
	new_pc->SetTransform(latest_alignment);
	target.insert({latest_ts, new_pc});
}

AlignedTrajectoryOutput::AlignedTrajectoryOutput(const std::string &name, AlignmentOutput *alignment, BaseOutput *trajectory_output) : DerivedOutput(name, values::VT_TRAJECTORY, {alignment, trajectory_output}), alignment_(alignment), trajectory_(trajectory_output)
{
	SetKeepOnlyMostRecent(true);
}

AlignedTrajectoryOutput::~AlignedTrajectoryOutput()
{
	SetKeepOnlyMostRecent(true);
}

void AlignedTrajectoryOutput::Recalculate()
{
	assert(GetKeepOnlyMostRecent());
	auto &target = GetCachedValueMap();

	for(auto i : target) {
		delete i.second;
	}
	target.clear();

	if(!trajectory_->IsActive() || trajectory_->GetValues().empty() || !alignment_->IsActive() || alignment_->GetValues().empty()) {
		return;
	}

	auto latest_data_point = trajectory_->GetMostRecentValue();
	auto latest_trajectory = (values::TrajectoryValue*)latest_data_point.second;
	auto latest_ts = latest_data_point.first;

	auto latest_alignment = ((values::TypedValue<Eigen::Matrix4f>*)alignment_->GetMostRecentValue().second)->GetValue();

	values::Trajectory *t = new values::Trajectory();
	for (auto &point : latest_trajectory->GetPoints()) {
		auto &pose = point.second.GetValue();

		if ((pose.array().abs() == std::numeric_limits<float>::infinity()).any())
			continue;

		Eigen::Matrix4f transformed_pose = latest_alignment * pose;

		t->push_back(point.first, values::PoseValue(transformed_pose));
	}

	auto new_trajectory = new values::TrajectoryValue(*t);
	target.insert({latest_ts, new_trajectory});
}


PointCloudHeatMap::PointCloudHeatMap(const std::string &name,
				     BaseOutput *gt_pointcloud, BaseOutput *pointcloud,
				     const std::function<values::ColoredPoint3DF(const values::HeatMapPoint3DF&, double, double)> &convert) :
        DerivedOutput(name, values::VT_HEATMAPPOINTCLOUD, {gt_pointcloud, pointcloud}, false),
        gt_pointcloud_(gt_pointcloud), pointcloud_(pointcloud), convert(convert)
{ }

slambench::values::HeatMapPointCloudValue *getValue(const pcl::PointCloud<point_t>::Ptr &gt,
						    const pcl::PointCloud<point_t>::Ptr &test) {

	auto value = new slambench::values::HeatMapPointCloudValue();

	pcl::search::KdTree<point_t>::Ptr tree (new pcl::search::KdTree<point_t>);
	tree->setInputCloud(gt);

	std::vector<int> pointIdxNKNSearch(1);
	std::vector<float> pointNKNSquaredDistance(1);

	for(size_t i = 0; i < test->size(); i++) {
		const auto &point = test->at(i);
		tree->nearestKSearch(point, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
		const double distance = sqrt(pointNKNSquaredDistance.at(0));
		value->AddPoint(slambench::values::HeatMapPoint3DF(point.x, point.y, point.z, distance));
	}

	return value;
}

void PointCloudHeatMap::Recalculate()
{
	assert(GetKeepOnlyMostRecent());
	BaseOutput::value_map_t &target = GetCachedValueMap();

	for(auto i : target) {
		delete i.second;
	}
	target.clear();

	if (pointcloud_->Empty()) {
		return;
	}

	const BaseOutput::value_map_t::value_type &tested_frame = pointcloud_->GetMostRecentValue();
	const BaseOutput::value_map_t::value_type &gt_frame = gt_pointcloud_->GetMostRecentValue();

	const PointCloudValue *tested_pointcloud = reinterpret_cast<const PointCloudValue*>(tested_frame.second);
	const PointCloudValue *gt_pointcloud = reinterpret_cast<const PointCloudValue*>(gt_frame.second);

	pcl::PointCloud<point_t>::Ptr tested_cloud = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>);

	for (const auto &point : tested_pointcloud->GetPoints()) {
		const Eigen::Matrix4f &transform = tested_pointcloud->GetTransform();
		const Eigen::Vector4f eigenPoint(point.X, point.Y, point.Z, 1);
		const Eigen::Vector4f transformedPoint = transform * eigenPoint;

		tested_cloud->push_back({transformedPoint(0), transformedPoint(1), transformedPoint(2)});
	}

	pcl::PointCloud<point_t>::Ptr gt_cloud = pcl::PointCloud<point_t>::Ptr(new pcl::PointCloud<point_t>);
	for (const auto &point : gt_pointcloud->GetPoints()) {
		gt_cloud->push_back({point.X, point.Y, point.Z});
	}

	target.insert({tested_frame.first, getValue(gt_cloud, tested_cloud)});
}

PoseToXYZOutput::PoseToXYZOutput(BaseOutput* pose_output) : BaseOutput(pose_output->GetName() + " (XYZ)", values::ValueDescription({{"X", values::VT_DOUBLE}, {"Y", values::VT_DOUBLE}, {"Z", values::VT_DOUBLE}})), pose_output_(pose_output)
{

}

const BaseOutput::value_map_t& PoseToXYZOutput::GetValues() const
{
	GetMostRecentValue();
	return cached_values_;
}

const BaseOutput::value_map_t::value_type& PoseToXYZOutput::GetMostRecentValue() const
{
	cached_values_.clear();

	auto pose_value = pose_output_->GetMostRecentValue();
	auto val = (values::TypeForVT<values::VT_POSE>::type*)pose_value.second;

	float x = val->GetValue()(0, 3);
	float y = val->GetValue()(1, 3);
	float z = val->GetValue()(2, 3);

	auto xv = new values::TypeForVT<values::VT_DOUBLE>::type(x);
	auto yv = new values::TypeForVT<values::VT_DOUBLE>::type(y);
	auto zv = new values::TypeForVT<values::VT_DOUBLE>::type(z);

	cached_values_.insert({pose_value.first, new values::TypeForVT<values::VT_COLLECTION>::type({{"X", xv}, {"Y", yv}, {"Z", zv}})});
	return *cached_values_.begin();
}

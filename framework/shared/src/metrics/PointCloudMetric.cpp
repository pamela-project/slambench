#include "metrics/PointCloudMetric.h"
#include "io/format/PointCloud.h"

using slambench::values::PointCloudValue;
using namespace slambench::metrics;

PointCloudMetric::PointCloudMetric(const slambench::outputs::BaseOutput * const heatmap_pointcloud) :
    Metric("PointCloud_Metric"), heatmap_pointcloud(heatmap_pointcloud)
{ }

const slambench::values::ValueDescription &PointCloudMetric::GetValueDescription() const {
	static const slambench::values::ValueDescription desc = slambench::values::ValueDescription({
		{"PointCloud_Metric", slambench::values::VT_DOUBLE}});
	return desc;
}

const std::string& PointCloudMetric::GetDescription() const {
	static std::string desc = "Point Cloud Reconstruction Accuracy";
	return desc;
}

void PointCloudMetric::MeasureStart(Phase* /* unused */) { }

void PointCloudMetric::MeasureEnd(Phase* /* unused */) { }

constexpr int COMPUTE_EVERY_FRAMES = 20;

Value *PointCloudMetric::GetValue(Phase* /* unused */) {

	if (heatmap_pointcloud->Empty())
	    return new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(std::nan(""));

	const outputs::Output::value_map_t::value_type tested_frame = heatmap_pointcloud->GetMostRecentValue();
	const auto pcv = reinterpret_cast<const values::HeatMapPointCloudValue*>(tested_frame.second);

	double total = 0;
	for (const auto &value : pcv->GetPoints()) {
		total += value.value;
	}

	const double average = total / pcv->GetPoints().size();

	return new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(average);

}


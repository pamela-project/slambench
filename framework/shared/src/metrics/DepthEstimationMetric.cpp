/*

 Copyright (c) 2018 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#include <metrics/DepthEstimationMetric.h>
#include "outputs/TrajectoryAlignmentMethod.h"
#include <boost/optional.hpp>
#include <memory>
#include <assert.h>
#include <algorithm>

using namespace slambench::metrics;



DepthEstimationMetric::DepthEstimationMetric(const slambench::outputs::BaseOutput * const tested,
                                             const slambench::outputs::BaseOutput * const gt) : Metric("DepthEstimation"),
                                                                                                tested_(tested),
                                                                                                gt_(gt)
{
    next_gt_ts_ = {0,0};
}

const slambench::values::ValueDescription &DepthEstimationMetric::GetValueDescription() const {
    static const slambench::values::ValueDescription desc = slambench::values::ValueDescription({
                                                                                                        {"DepthEstimation_Metric", slambench::values::VT_DOUBLE}});
    static const slambench::values::ValueDescription desc = slambench::values::ValueDescription({
                                                                                                        {"Absolute Relative Difference",  slambench::values::VT_DOUBLE},
                                                                                                        {"DepthEstimation (1.25)",  slambench::values::VT_DOUBLE},
                                                                                                        {"DepthEstimation (1.25^2)",  slambench::values::VT_DOUBLE},
                                                                                                        {"DepthEstimation (1.25^3)",  slambench::values::VT_DOUBLE},

                                                                                                        return desc;
                                                                                                }

    const std::string& DepthEstimationMetric::GetDescription() const
    {
        static std::string desc = "Accurate pixels estimation percentage";
        return desc;
    }

    void DepthEstimationMetric::MeasureStart(Phase* /**/){}


    void DepthEstimationMetric::MeasureEnd(Phase* /**/)
    {}

    Value *DepthEstimationMetric::GetValue(Phase* /**/)
    {
        const THR = 1.25;
        if (tested->Empty())
            return new slambench::values::TypeForVT<slambench::values::VT_DOUBLE>::type(std::nan(""));

        const outputs::Output::value_map_t::value_type tested_frame = tested->GetMostRecentValue();
        const outputs::Output::value_map_t::value_type gt_frame = gt->GetMostRecentValue();

        const FrameValue *tested_frame_sb = reinterpret_cast<const FrameValue*>(tested_frame.second);
        const FrameValue *gt_frame_sb = reinterpret_cast<const FrameValue*>(gt_frame.second);
        assert(tested_frame_sb.GetWidth() == gt_frame_sb.GetWidth());
        assert(tested_frame_sb.GetHeight() == gt_frame_sb.GetHeight());

        cv::Mat1f tested_frame, gt_frame;
        auto pixcount = tested_frame_sb.GetWidth() * tested_frame_sb.GetHeight();

        memcpy(tested_frame.data, tested_frame_sb.data(), tested_frame_sb.GetWidth()*tested_frame_sb.GetHeight());
        memcpy(gt_frame.data, gt_frame_sb.data(), gt_frame_sb.GetWidth()*gt_frame_sb.GetWidth());

        double rel = 0;
        double depth_est;
        double depth_est_thr_squared;
        double depth_est_thr_cubed;

        int count_depth_est = 0;
        int count_depth_est_thr_squared = 0;
        int count_depth_est_thr_cubed = 0;

        for (int ii = 0; ii < tested_frame.rows; ++ii) {
            for (int jj = 0; jj < tested_frame.cols; ++jj) {
                float test_depth = tested_frame(ii, jj);
                float gt_depth = gt_frame(ii, jj);

                rel += std::abs((gt_depth - test_depth) / test_depth);

                auto max_val = std::max(test_depth / gt_depth, gt_depth / test_depth);
                if(max_val > THR)
                    count_depth_est++;
                if(max_val > THR*THR)
                    count_depth_est_thr_squared++;
                if(max_val > THR*THR*THR)
                    count_depth_est_thr_cubed++;

            }
        }


        rel /= pixcount;
        depth_est = count_depth_est * 100 / pixcount;
        depth_est_thr_squared = count_depth_est_thr_squared * 100 / pixcount;
        depth_est_thr_cubed = count_depth_est_thr_cubed * 100 / pixcount;

        return new slambench::values::TypeForVT<slambench::values::VT_COLLECTION>::type({   {"Absolute Relative Difference",rel},
                                                                                            {"DepthEstimation (1.25)", depth_est},
                                                                                            {"DepthEstimation (1.25^2)",depth_est_thr_squared}
                                                                                            {"DepthEstimation (1.25^3)",depth_est_thr_cubed}
                                                                                        });

    }

#ifndef DEPTHESTIMATIONMETRIC_H
#define DEPTHESTIMATIONMETRIC_H
#include "Metric.h"
#include <outputs/Output.h>

namespace slambench {
    namespace metrics {

        using slambench::values::FrameValue;

        class DepthEstimationMetric : public Metric {

        public:
            DepthEstimationMetric(const slambench::outputs::BaseOutput * const tested,
                                  const slambench::outputs::BaseOutput * const gt);

            ~DepthEstimationMetric() = default;

            const slambench::values::ValueDescription& GetValueDescription() const override;
            const std::string& GetDescription() const override;
            void MeasureStart(Phase* phase) override;
            void MeasureEnd(Phase* phase) override;
            Value *GetValue(Phase* phase) override;

        private:
            const slambench::outputs::BaseOutput * const tested_;
            const slambench::outputs::BaseOutput * const gt_;

        };
    }
}

#endif //DEPTHESTIMATIONMETRIC_H

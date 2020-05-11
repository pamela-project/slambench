#ifndef POINTCLOUDMETRIC_H
#define POINTCLOUDMETRIC_H

#include "metrics/Metric.h"
#include "outputs/Output.h"

namespace slambench {
    namespace metrics {

        using slambench::values::PointCloudValue;

        class PointCloudMetric : public Metric {
            private:
                const slambench::outputs::BaseOutput * const heatmap_pointcloud;

                slambench::TimeStamp lastMeasurement;

            public:
                PointCloudMetric(const slambench::outputs::BaseOutput * const heatmap_pointcloud);

                ~PointCloudMetric() = default;

                const slambench::values::ValueDescription& GetValueDescription() const override;
                const std::string& GetDescription() const override;

                void MeasureStart(Phase* phase) override;
                void MeasureEnd(Phase* phase) override;

                Value *GetValue(Phase* phase) override;
        };
    }
}

#endif

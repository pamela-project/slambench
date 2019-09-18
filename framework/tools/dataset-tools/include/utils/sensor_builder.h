/*

 Copyright (c) 2019 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_DATASET_BUILDER_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_DATASET_BUILDER_H_

#include <io/sensor/AccelerometerSensor.h>

namespace slambench {
  namespace io {

    template <typename T>
    class SensorBuilder {
     protected:
      std::string name_;
      std::string description_;

      float rate_;
      int width_;
      int height_;

      frameformat::EFrameFormat frameFormat_;
      pixelformat::EPixelFormat pixelFormat_;

      Sensor::pose_t pose_;
      CameraSensor::intrinsics_t intrinsics_;
      CameraSensor::distortion_type_t distortion_type_ = CameraSensor::NoDistortion;
      CameraSensor::distortion_coefficients_t distortion_;

      DepthSensor::disparity_type_t disparity_type_;
      DepthSensor::disparity_params_t disparity_;

      SensorBuilder() = default;
    public:
      T& name(const std::string& name) {
        name_ = name;
        return static_cast<T&>(*this);
      }

      T& description(const std::string& description) {
        description_ = description;
        return static_cast<T&>(*this);
      }

      T& rate(float rate) {
        rate_ = rate;
        return static_cast<T&>(*this);
      }

      T& size(uint32_t width, uint32_t height) {
        width_ = width;
        height_ = height;
        return static_cast<T&>(*this);
      }

      T& frameFormat(const frameformat::EFrameFormat& format) {
        frameFormat_ = format;
        return static_cast<T&>(*this);
      }

      T& pixelFormat(const pixelformat::EPixelFormat& format) {
        pixelFormat_ = format;
        return static_cast<T&>(*this);
      }

      T& pose(const Sensor::pose_t& pose) {
        pose_ = pose;
        return static_cast<T&>(*this);
      }

      T& intrinsics(const CameraSensor::intrinsics_t& intrinsics) {
        for(unsigned int i = 0; i < 4 ; ++i) {
          intrinsics_[i] = intrinsics[i];
        }
        return static_cast<T&>(*this);
      }

      T& distortion(const CameraSensor::distortion_type_t& type,
                    const CameraSensor::distortion_coefficients_t& distortion) {
        distortion_type_ = type;
        for(unsigned int i = 0; i < 5 ; ++i) {
          distortion_[i] = distortion[i];
        }
        return static_cast<T&>(*this);
      }

      T& disparity(const DepthSensor::disparity_type_t& type,
                   const DepthSensor::disparity_params_t& disparity) {
        disparity_type_ = type;
        for(int i = 0; i < 2; ++i) {
          disparity_[i] = disparity[i];
        }
        return static_cast<T&>(*this);
      }
    };

    class CameraSensorBuilder : public SensorBuilder<CameraSensorBuilder> {
     public:
      CameraSensor* build() {
        auto sensor = new CameraSensor(name_.empty() ? "Camera" : name_,
                                       CameraSensor::kCameraType);
        sensor->Rate = rate_;
        sensor->Width = width_;
        sensor->Height = height_;
        sensor->FrameFormat = frameFormat_;
        sensor->PixelFormat = pixelFormat_;
        sensor->Description = description_.empty() ? "Camera" : description_;
        sensor->CopyPose(pose_);
        sensor->CopyIntrinsics(intrinsics_);

        sensor->DistortionType = distortion_type_;

        if (distortion_type_ == CameraSensor::RadialTangential) {
          sensor->CopyRadialTangentialDistortion(distortion_);
        }

        if (distortion_type_ == CameraSensor::Equidistant) {
          sensor->CopyEquidistantDistortion(distortion_);
        }

        return sensor;
      }
    };

    class RGBSensorBuilder : public SensorBuilder<RGBSensorBuilder> {
     public:
      CameraSensor* build() {

        std::string name =  name_.empty() ? "RGB" : name_;
        auto sensor = new CameraSensor(name, CameraSensor::kCameraType);
        sensor->Rate = rate_;
        sensor->Width = width_;
        sensor->Height = height_;
        sensor->FrameFormat = frameformat::Raster;
        sensor->PixelFormat = pixelformat::RGB_III_888;
        sensor->Description = description_.empty() ? "RGB" : description_;
        sensor->CopyPose(pose_);
        sensor->CopyIntrinsics(intrinsics_);

        sensor->DistortionType = distortion_type_;

        if (distortion_type_ == CameraSensor::RadialTangential) {
          sensor->CopyRadialTangentialDistortion(distortion_);
        }

        if (distortion_type_ == CameraSensor::Equidistant) {
          sensor->CopyEquidistantDistortion(distortion_);
        }

        return sensor;
      }
    };

    class GreySensorBuilder : public SensorBuilder<GreySensorBuilder> {

     public:
      CameraSensor* build() {
        std::string name =  name_.empty() ? "Grey" : name_;
        auto sensor = new CameraSensor(name, CameraSensor::kCameraType);
        sensor->Rate = rate_;
        sensor->Width = width_;
        sensor->Height = height_;
        sensor->FrameFormat = frameformat::Raster;
        sensor->PixelFormat = pixelformat::G_I_8;
        sensor->Description = description_.empty() ? "Grey" : description_;
        sensor->CopyPose(pose_);
        sensor->CopyIntrinsics(intrinsics_);

        sensor->DistortionType = distortion_type_;

        if (distortion_type_ == CameraSensor::RadialTangential) {
          sensor->CopyRadialTangentialDistortion(distortion_);
        }

        if (distortion_type_ == CameraSensor::Equidistant) {
          sensor->CopyEquidistantDistortion(distortion_);
        }

        return sensor;
      }
    };

    class DepthSensorBuilder : public SensorBuilder<DepthSensorBuilder> {

     public:
      DepthSensor* build() {
        std::string name =  name_.empty() ? "Depth" : name_;
        auto sensor = new DepthSensor(name);

        sensor->Rate = rate_;
        sensor->Width = width_;
        sensor->Height = height_;
        sensor->FrameFormat = frameformat::Raster;
        sensor->PixelFormat = pixelformat::D_I_16;
        sensor->Description = description_.empty() ? "Depth" : description_;
        sensor->CopyPose(pose_);
        sensor->CopyIntrinsics(intrinsics_);
        sensor->DistortionType = distortion_type_;

        if (distortion_type_ == CameraSensor::RadialTangential) {
          sensor->CopyRadialTangentialDistortion(distortion_);
        }

        if (distortion_type_ == CameraSensor::Equidistant) {
          sensor->CopyEquidistantDistortion(distortion_);
        }

        sensor->DisparityType = disparity_type_;
        sensor->CopyDisparityParams(disparity_);

        return sensor;
      }
    };

    class AccSensorBuilder : public SensorBuilder<AccSensorBuilder> {
     public:
      AccelerometerSensor* build() {
        std::string name =  name_.empty() ? "Accelerometer" : name_;
        auto sensor = new AccelerometerSensor(name);
        sensor->Description = description_.empty() ? "Accelerometer" : description_;
        return sensor;
      }
    };

    class GTSensorBuilder : public SensorBuilder<GTSensorBuilder> {
     public:
      GroundTruthSensor* build() {
        std::string name =  name_.empty() ? "GroundTruth" : name_;
        auto sensor = new GroundTruthSensor(name);
        sensor->Description = description_.empty() ? "GroundTruth" : description_;
        sensor->Rate = rate_;
        sensor->CopyPose(pose_);
        return sensor;
      }
    };

  }  // namespace io
}  // namespace slambench

#endif  // FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_DATASET_BUILDER_H_

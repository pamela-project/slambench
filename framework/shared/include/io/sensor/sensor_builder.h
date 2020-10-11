/*

 Copyright (c) 2019 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_SENSOR_SENSOR_BUILDER_H
#define IO_SENSOR_SENSOR_BUILDER_H
#include <iostream>
#include <io/sensor/DepthSensor.h>
#include <io/sensor/CameraSensor.h>
#include <io/sensor/GroundTruthSensor.h>
#include <io/sensor/EventCameraSensor.h>
#include <io/sensor/AccelerometerSensor.h>
#include <io/sensor/IMUSensor.h>
#include <io/sensor/GyroSensor.h>
#include <io/sensor/OdomSensor.h>

namespace slambench {
  namespace io {

    template <typename T>
    class SensorBuilder {
     protected:
      std::string name_;
      std::string description_;
      uint8_t index_;

      float rate_;
      float delay_;
      int width_;
      int height_;

      frameformat::EFrameFormat frameFormat_;
      pixelformat::EPixelFormat pixelFormat_;

      Sensor::pose_t pose_ = Sensor::pose_t::Identity();
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
      T& index(const uint8_t& index) {
          index_ = index;
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

      T& delay(const float& delay) {
        delay_ = delay;
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
      static bool check(Sensor* s, std::string& name)
      {
          if(!s) {
              std::cout << name << " sensor not found..." << std::endl;
              return false;
          } else {
              std::cout << s->GetName() << " sensor created..." << std::endl;
              return true;
          }
      }
    };

    class CameraSensorBuilder : public SensorBuilder<CameraSensorBuilder> {
     public:
      CameraSensor* build() {
        std::string name =  name_.empty() ? "Camera" : name_;
        auto sensor = new CameraSensor(name, CameraSensor::kCameraType);
        sensor->Rate = rate_;
        sensor->Width = width_;
        sensor->Height = height_;
        sensor->Index = index_;
        sensor->FrameFormat = frameFormat_;
        sensor->PixelFormat = pixelFormat_;
        sensor->Description = description_.empty() ? "Camera" : description_;
        sensor->CopyPose(pose_);
        sensor->CopyIntrinsics(intrinsics_);

        sensor->DistortionType = distortion_type_;
        sensor->CopyDistortion(distortion_,distortion_type_);
        check(sensor, name);
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
        sensor->FrameFormat = frameFormat_ ? frameFormat_ : frameformat::Raster;
        sensor->PixelFormat = pixelFormat_ ? pixelFormat_ : pixelformat::RGB_III_888;
        sensor->Description = description_.empty() ? "RGB" : description_;
        sensor->CopyPose(pose_);
        sensor->CopyIntrinsics(intrinsics_);

        sensor->DistortionType = distortion_type_;
        sensor->CopyDistortion(distortion_,distortion_type_);
        check(sensor, name);
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
        sensor->FrameFormat = frameFormat_ ? frameFormat_ : frameformat::Raster;
        sensor->PixelFormat = pixelformat::G_I_8;
        sensor->Description = description_.empty() ? "Grey" : description_;
        sensor->CopyPose(pose_);
        sensor->CopyIntrinsics(intrinsics_);

        sensor->DistortionType = distortion_type_;
        sensor->CopyDistortion(distortion_,distortion_type_);

        check(sensor, name);
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
        sensor->PixelFormat = pixelFormat_ ? pixelFormat_ : pixelformat::D_I_16;
        sensor->Description = description_.empty() ? "Depth" : description_;
        sensor->CopyPose(pose_);
        sensor->CopyIntrinsics(intrinsics_);
        sensor->DistortionType = distortion_type_;
        sensor->CopyDistortion(distortion_,distortion_type_);

        sensor->DisparityType = disparity_type_;
        sensor->CopyDisparityParams(disparity_);
        check(sensor, name);
        return sensor;
      }
    };

    class AccSensorBuilder : public SensorBuilder<AccSensorBuilder> {
     public:
      AccelerometerSensor* build() {
        std::string name =  name_.empty() ? "Accelerometer" : name_;
        auto sensor = new AccelerometerSensor(name);
        sensor->Description = description_.empty() ? "Accelerometer" : description_;
        check(sensor, name);
        return sensor;
      }
    };

    class GyroSensorBuilder : public SensorBuilder<GyroSensorBuilder> {
     public:
        GyroSensor* build() {
        std::string name =  name_.empty() ? "Gyro" : name_;
        auto sensor = new GyroSensor(name);
        sensor->Description = description_.empty() ? "Gyroscope" : description_;
        check(sensor, name);
        return sensor;
      }
    };


    class OdomSensorBuilder : public SensorBuilder<OdomSensorBuilder> {
     public:
        OdomSensor* build() {
        std::string name =  name_.empty() ? "Odom" : name_;
        auto sensor = new OdomSensor(name);
        sensor->Description = description_.empty() ? "Odometry" : description_;
        check(sensor, name);
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
        check(sensor, name);
        return sensor;
      }
    };

  }  // namespace io
}  // namespace slambench

#endif  // IO_SENSOR_SENSOR_BUILDER_H

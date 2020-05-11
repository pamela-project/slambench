/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef OUTPUT_VALUE_H
#define OUTPUT_VALUE_H

#include "TimeStamp.h"
#include "io/PixelFormat.h"

#include <map>
#include <memory>
#include <vector>

#include <Eigen/Eigen>

namespace slambench {
	namespace values {
		class ConstValueDispatch;
		class ValueDispatch;

		enum ValueType {
			VT_UNKNOWN, // Represents an initialised or invalid VT

			VT_COLLECTION, // Can represent a structured collection of values of other types
			VT_LIST, // Can represent a flat list of values of other types

			VT_U64, // Can represent an arbitrary 64-bit value
			VT_DOUBLE, // Can represent an arbitrary double-precision float
			VT_STRING, // Can represent a text string
			VT_TRAJECTORY, // Represents a sequence of poses
			VT_POSE, // A matrix which specifically represents a pose (position + orientation)
			VT_POINTCLOUD, // A list of point positions
			VT_COLOUREDPOINTCLOUD, // A list of point positions with colours
			VT_HEATMAPPOINTCLOUD, // A pointcloud for visualising values
			VT_FEATURE, // A tracked feature
			VT_FEATURELIST,
			VT_FRAME,
			VT_MATRIX //Can represent an arbitrary matrix
		};

		static inline const std::string TypeAsString (const ValueType& v) {
			switch (v)  {
			case VT_COLLECTION  : return "VT_COLLECTION  ";
			case VT_LIST        : return "VT_LIST        ";
			case VT_U64         : return "VT_U64         ";
			case VT_DOUBLE      : return "VT_DOUBLE      ";
			case VT_STRING      : return "VT_STRING      ";
			case VT_TRAJECTORY  : return "VT_TRAJECTORY  ";
			case VT_POSE        : return "VT_POSE        ";
			case VT_POINTCLOUD  : return "VT_POINTCLOUD  ";
			case VT_COLOUREDPOINTCLOUD  : return "VT_COLOUREDPOINTCLOUD  ";
			case VT_HEATMAPPOINTCLOUD   : return "VT_HEATMAPPOINTCLOUD  ";
			case VT_FEATURE     : return "VT_FEATURE     ";
			case VT_FEATURELIST : return "VT_FEATURELIST ";
			case VT_FRAME       : return "VT_FRAME       ";
			case VT_MATRIX      : return "VT_MATRIX      ";
			case VT_UNKNOWN     :
			default             : return "VT_UNKNOWN     ";

			}
		}

		class ValueDescription {
		public:
			typedef std::vector<std::pair<std::string, ValueDescription>> structured_description;

			ValueDescription(ValueType type);
			ValueDescription(const structured_description &structured_desc);

			ValueType GetType() const;
			const structured_description &GetStructureDescription() const;
		private:
			ValueType type_;
			structured_description structured_type_;
		};

		class Value {
		public:
			Value(ValueType type) : value_type_(type) {}

			ValueType GetType() const { return value_type_; }
			virtual ~Value();

			void Dispatch(ValueDispatch *vd);
			void Dispatch(ConstValueDispatch *vd) const;

		private:
			ValueType value_type_;
		};

		// Fail if no specializations are provided
		template<ValueType T> struct TypeForVT;
		template<typename T> struct VTForType;

		template<typename T> class TypedValue : public Value {
		public:
			TypedValue() : Value(VTForType<T>::value()) {}
			TypedValue(const T& val) : Value(VTForType<T>::value()), value_(val) {}

			const T &GetValue() const { return value_; }
		private:
			T value_;
		};

		class ValueCollectionValue : public Value {
		public:
			typedef std::vector<std::pair<std::string, Value*>> value_map_t;

			ValueCollectionValue() : Value(VT_COLLECTION) {}
			ValueCollectionValue(const std::initializer_list<value_map_t::value_type> &values) : Value(VT_COLLECTION), values_(values) {}
			~ValueCollectionValue();

			const value_map_t &GetValue() const { return values_; }

		private:
			value_map_t values_;
		};

		class ValueListValue : public Value {
		public:
			typedef std::vector<Value*> value_list;

			ValueListValue() : Value(VT_LIST) {}
			ValueListValue(const std::initializer_list<Value*> &values) : Value(VT_LIST), values_(values) {}
			ValueListValue(const std::vector<Value*> &values) : Value(VT_LIST), values_(values) {}
			~ValueListValue();

			const value_list &GetValue() const { return values_; }

		private:
			value_list values_;
		};

		class PoseValue : public Value {
		public:
			PoseValue(const Eigen::Matrix4f &pose) : Value(VT_POSE), pose_(pose) {}
            Eigen::Vector3f GetTranslation() const { return pose_.block<3,1>(0,3);	}
            Eigen::Matrix3f GetRotation() const	{ return pose_.block<3,3>(0,0);}
            const Eigen::Matrix4f &GetValue() const { return pose_; }
		private:
			Eigen::Matrix4f pose_;
		};

		class Trajectory {
		public:
			typedef std::pair<TimeStamp, PoseValue> value_t;
		private:
			typedef std::vector<value_t> storage_t;
		public:
			typedef storage_t::const_iterator const_iterator_t;
			typedef storage_t::const_reverse_iterator const_reverse_iterator_t;

			const value_t &at(unsigned int i) const {
				return values_.at(i);
			}

			const PoseValue &at(const TimeStamp &ts) const {
				//todo: search more intelligently
				for(auto &i : values_) {
					if(i.first == ts) {
						return i.second;
					}
				}
				throw std::out_of_range("Could not find timestamp in trajectory");
			}
			void push_back(const TimeStamp &ts, const PoseValue &pose) {
				if(!empty() && ts < values_.rbegin()->first) {
				  throw std::invalid_argument("Trajectory is append-only (inserted timestamp is before most recent recorded point)");
				}
				values_.push_back(value_t(ts, pose));
			}
			void insert(const value_t &value) { push_back(value.first, value.second); }

			size_t size() const { return values_.size(); }
			bool empty() const { return values_.size() == 0; }

			void clear() { values_.clear(); }

			const_iterator_t begin() const { return values_.begin(); }
			const_reverse_iterator_t rbegin() const { return values_.rbegin(); }
			const_iterator_t end() const { return values_.end(); }
			const_reverse_iterator_t rend() const { return values_.rend(); }
		private:
			std::vector<std::pair<TimeStamp, PoseValue>> values_;
		};

		class TrajectoryValue : public Value {
		public:
			typedef Trajectory pose_container_t;

			TrajectoryValue() : Value(VT_TRAJECTORY) {}
			TrajectoryValue(const pose_container_t &trajectory) : Value(VT_TRAJECTORY), poses_(trajectory) {}
			const pose_container_t &GetPoints() const { return poses_; }
		private:
			pose_container_t poses_;
		};

		class Point3DF {
		public:
			Point3DF() : X(0), Y(0), Z(0) {}
			Point3DF(float x, float y, float z) : X(x), Y(y), Z(z) {}
			float X, Y, Z;
		};

		class ColoredPoint3DF {
		public:
			ColoredPoint3DF() : X(0), Y(0), Z(0) , R(0), G(0), B(0) {}
			ColoredPoint3DF(float x, float y, float z) : X(x), Y(y), Z(z), R(0), G(0), B(0) {}
			ColoredPoint3DF(float x, float y, float z, uint8_t r, uint8_t g, uint8_t b) : X(x), Y(y), Z(z), R(r), G(g), B(b) {}
			float X, Y, Z;
			uint8_t R, G, B;
		};

		class HeatMapPoint3DF {
		public:
			HeatMapPoint3DF() : X(0), Y(0), Z(0) , value(0) {}
			HeatMapPoint3DF(float x, float y, float z, double value) : X(x), Y(y), Z(z), value(value) {}
			float X, Y, Z;
			double value;
		};



		class PointCloudValue : public Value {
		public:
			typedef std::vector<Point3DF> point_container_t;

			PointCloudValue() : Value(VT_POINTCLOUD), points_(std::make_shared<point_container_t>()), transform_(Eigen::Matrix4f::Identity()) { }
			PointCloudValue(const PointCloudValue &other) : Value(VT_POINTCLOUD), points_(other.points_), transform_(other.transform_) { }
			virtual ~PointCloudValue() { }

			void AddPoint(const Point3DF &point) { makeUnique(); points_->push_back(point);  }
			void Clear() { makeUnique(); points_->clear(); }

			const point_container_t &GetPoints() const { assert(points_); return *points_; }

			const Eigen::Matrix4f &GetTransform() const { return transform_; }
			void SetTransform(const Eigen::Matrix4f &new_txfm) { transform_ = new_txfm; }

		private:
			void makeUnique() { if(!points_.unique()) { points_ = std::make_shared<point_container_t>(*points_); } }

			std::shared_ptr<point_container_t> points_;
			Eigen::Matrix4f transform_;
		};



		class HeatMapPointCloudValue : public Value {
		public:
			typedef std::vector<HeatMapPoint3DF> point_container_t;

			HeatMapPointCloudValue() : Value(VT_HEATMAPPOINTCLOUD), points_(std::make_shared<point_container_t>()), transform_(Eigen::Matrix4f::Identity()) { }
			HeatMapPointCloudValue(const HeatMapPointCloudValue &other) : Value(VT_HEATMAPPOINTCLOUD), points_(other.points_), transform_(other.transform_) { }
			virtual ~HeatMapPointCloudValue() { }

			void AddPoint(const HeatMapPoint3DF &point) { makeUnique(); points_->push_back(point);  }
			void Clear() { makeUnique(); points_->clear(); }

			const point_container_t &GetPoints() const { assert(points_); return *points_; }

			const Eigen::Matrix4f &GetTransform() const { return transform_; }
			void SetTransform(const Eigen::Matrix4f &new_txfm) { transform_ = new_txfm; }

		private:
			void makeUnique() { if(!points_.unique()) { points_ = std::make_shared<point_container_t>(*points_); } }

			std::shared_ptr<point_container_t> points_;
			Eigen::Matrix4f transform_;
		};


		class ColoredPointCloudValue : public Value {
		public:
			typedef std::vector<ColoredPoint3DF> point_container_t;

			ColoredPointCloudValue() : Value(VT_COLOUREDPOINTCLOUD), points_(std::make_shared<point_container_t>()), transform_(Eigen::Matrix4f::Identity()) { }
			ColoredPointCloudValue(const ColoredPointCloudValue &other) : Value(VT_COLOUREDPOINTCLOUD), points_(other.points_), transform_(other.transform_) { }
			ColoredPointCloudValue(const HeatMapPointCloudValue &other, std::function<ColoredPoint3DF(const HeatMapPoint3DF&, double, double)> convert) :
						Value(VT_HEATMAPPOINTCLOUD),
						points_(std::make_shared<point_container_t>()),
						transform_(Eigen::Matrix4f::Identity()) {
				const auto otherValues = other.GetPoints();
				points_->reserve(otherValues.size());
				double max = 0, min = std::numeric_limits<double>::max();
				for (const auto &otherValue : otherValues) {
					min = std::min(otherValue.value, min);
					max = std::max(otherValue.value, max);
				}
				for (const auto &otherValue : otherValues)
					points_->push_back(convert(otherValue, min, max));
			}
			virtual ~ColoredPointCloudValue() { }

			void AddPoint(const ColoredPoint3DF &point) { makeUnique(); points_->push_back(point);  }
			void Clear() { makeUnique(); points_->clear(); }

			const point_container_t &GetPoints() const { assert(points_); return *points_; }

			const Eigen::Matrix4f &GetTransform() const { return transform_; }
			void SetTransform(const Eigen::Matrix4f &new_txfm) { transform_ = new_txfm; }

		private:
			void makeUnique() { if(!points_.unique()) { points_ = std::make_shared<point_container_t>(*points_); } }

			std::shared_ptr<point_container_t> points_;
			Eigen::Matrix4f transform_;
		};


		class FrameValue : public Value {
		public:
			FrameValue(const FrameValue&);
			FrameValue(uint32_t width, uint32_t height, slambench::io::pixelformat::EPixelFormat pxl_format, void *data);
			FrameValue(uint32_t width, uint32_t height, slambench::io::pixelformat::EPixelFormat pxl_format);

			void *GetData() { return data_.data(); }
            const void *GetData() const { return data_.data(); }
            unsigned char at(int row, int col) const { return data_.at(row*height_+col); } //NOTE: assumes row-major
			uint32_t GetWidth() const { return width_; }
			uint32_t GetHeight() const { return height_; }
			slambench::io::pixelformat::EPixelFormat GetFormat() const { return pxl_format_; }

		private:
			uint32_t width_, height_;
			slambench::io::pixelformat::EPixelFormat pxl_format_;
			std::vector<unsigned char> data_;
		};

		class FeatureValue : public Value {
		public:
			FeatureValue(const Eigen::Matrix4f &pose, const FrameValue &patch);
			virtual ~FeatureValue();

			FrameValue &GetPatch() { return image_patch_; }
			Eigen::Matrix4f &GetPose() { return pose_; }

		private:
			FrameValue image_patch_;
			Eigen::Matrix4f pose_;
		};

		/* Specialisations of VT traits live down here */

		template<> struct TypeForVT<VT_COLLECTION> { typedef ValueCollectionValue type; };
		template<> struct TypeForVT<VT_U64> { typedef TypedValue<uint64_t> type; };
		template<> struct TypeForVT<VT_DOUBLE> { typedef TypedValue<double> type; };
		template<> struct TypeForVT<VT_STRING> { typedef TypedValue<std::string> type; };
		template<> struct TypeForVT<VT_TRAJECTORY> { typedef TrajectoryValue type; };
		template<> struct TypeForVT<VT_POSE> { typedef PoseValue type; };
		template<> struct TypeForVT<VT_POINTCLOUD> { typedef PointCloudValue type; };
		template<> struct TypeForVT<VT_MATRIX> { typedef TypedValue<Eigen::Matrix4f> type; };

		template<> struct VTForType<uint64_t> { static constexpr ValueType value() { return VT_U64; } };
		template<> struct VTForType<double> { static constexpr ValueType value() { return VT_DOUBLE; } };
		template<> struct VTForType<std::string> { static constexpr ValueType value() { return VT_STRING; } };
		template<> struct VTForType<Eigen::Matrix4f> { static constexpr ValueType value() { return VT_MATRIX; } };
	}
}

#endif /* OUTPUT_VALUE_H */


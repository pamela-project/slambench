/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef OUTPUT_H
#define OUTPUT_H

#include "TimeStamp.h"
#include "values/Value.h"
#include "TrajectoryAlignmentMethod.h"

#include <map>
#include <set>
#include <vector>

namespace slambench {
	namespace outputs {
		class TrajectoryInterface;
		
		/**
		 * Base output interface
		 */
		class BaseOutput {
		public:
			typedef std::function<void (const BaseOutput*)> callback_t;
			
			typedef slambench::TimeStamp timestamp_t;
			typedef std::map<timestamp_t, const values::Value*> value_map_t;
			
			BaseOutput(const std::string &name, const values::ValueDescription &type, bool is_main_output = false);
			virtual ~BaseOutput();
			
			const std::string &GetName() const { return name_; }
			values::ValueType GetType() const { return type_.GetType(); }
			const values::ValueDescription &GetValueDescription() const { return type_; }
			
			void SetKeepOnlyMostRecent(bool only_most_recent) { only_keep_most_recent_ = only_most_recent; }
			bool GetKeepOnlyMostRecent() const { return only_keep_most_recent_; }
			
			bool IsActive() const { return active_; }
			void SetActive(bool active) { active_ = active; }

			virtual const value_map_t &GetValues() const = 0;
			virtual const value_map_t::value_type &GetMostRecentValue() const;
			virtual bool Empty() const;
			
			bool IsMainOutput() const { return main_; }
			
			void AddUpdateCallback(callback_t callback) { update_callbacks_.push_back(callback); }
			void RemoveUpdateCallback(callback_t callback);
		protected:
			void Updated() const;
			
		private:
			const std::string name_;
			values::ValueDescription type_;
			bool only_keep_most_recent_;
			bool active_;
			bool main_;
			
			std::vector<callback_t> update_callbacks_;
		};
		
		/**
		 * Output which has values added to it and stores them in a map
		 */
		class Output : public BaseOutput {
		public:
			Output(const std::string &name, values::ValueType type, bool main_output = false);
			virtual ~Output() override;
			
			void AddPoint(timestamp_t time, const values::Value *value);

			const value_map_t &GetValues() const override;
			const value_map_t::value_type &GetMostRecentValue() const override;
		private:
			value_map_t  values_;
		};
		
		class DerivedOutput : public BaseOutput {
		public:
			DerivedOutput(const std::string &name, values::ValueType type, const std::initializer_list<BaseOutput*> &derived_from, bool main = false);
			virtual ~DerivedOutput();
			
			bool Empty() const override;
			const value_map_t::value_type& GetMostRecentValue() const override;
			const BaseOutput::value_map_t& GetValues() const override;

			void Invalidate();
			
		protected:
			virtual void Recalculate() = 0;
			value_map_t &GetCachedValueMap();
		private:
			void recalculate() const;
			value_map_t cached_values_;
			std::vector<BaseOutput*> derived_from_;
			bool up_to_date_;
		};
		
		class AlignmentOutput : public DerivedOutput {
		public:
			AlignmentOutput(const std::string &name, TrajectoryInterface *gt_trajectory, BaseOutput *trajectory, TrajectoryAlignmentMethod *method);
			virtual ~AlignmentOutput();
			
			void Recalculate() override;
		private:
			TrajectoryInterface *gt_trajectory_;
			BaseOutput *trajectory_;
			TrajectoryAlignmentMethod *method_;
		};
		
		class AlignedPoseOutput : public DerivedOutput {
		public:
			AlignedPoseOutput(const std::string &name, AlignmentOutput *alignment, BaseOutput *pose_output);
			virtual ~AlignedPoseOutput();
			
			void Recalculate() override;
		private:
			AlignmentOutput *alignment_;
			BaseOutput *pose_output_;
		};
		
		/**
		 * An output which returns a transformed point cloud
		 */
		class AlignedPointCloudOutput : public DerivedOutput {
		public:
			AlignedPointCloudOutput(const std::string &name, AlignmentOutput *, BaseOutput *pc_output);
			virtual ~AlignedPointCloudOutput();

			void Recalculate() override;

		private:
			AlignmentOutput *alignment_;
			BaseOutput *pointcloud_;
		};
		
		class PoseToXYZOutput : public BaseOutput {
		public:
			PoseToXYZOutput(BaseOutput *pose_output);
			virtual ~PoseToXYZOutput();
			const BaseOutput::value_map_t& GetValues() const override;
			const value_map_t::value_type& GetMostRecentValue() const override;
			
		private:
			BaseOutput *pose_output_;
			mutable value_map_t cached_values_;
		};
	}
}

#endif /* OUTPUT_H */


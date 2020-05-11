/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */



#ifndef TRAJECTORYINTERFACE_H
#define TRAJECTORYINTERFACE_H

#include "outputs/Output.h"
#include "values/Value.h"

namespace slambench {
	namespace outputs {
		
		/**
		 * We want to treat multiple things as trajectories so we need a way
		 * of interfacing with these things in a consistent way.		 
		 */
		class TrajectoryInterface {
		public:
			virtual values::PoseValue Get(const TimeStamp &when) const = 0;
			virtual values::TrajectoryValue::pose_container_t GetAll() const = 0;
		};
		
		class PoseOutputTrajectoryInterface : public TrajectoryInterface {
		public:
			PoseOutputTrajectoryInterface(BaseOutput *pose_output);
			virtual ~PoseOutputTrajectoryInterface();
			
			values::PoseValue Get(const TimeStamp& when) const override;
			values::TrajectoryValue::pose_container_t GetAll() const override;
			
		private:
			void recalculate() const;
			
			BaseOutput *pose_output_;
			
			mutable TimeStamp newest_point_;
			mutable values::TrajectoryValue::pose_container_t cached_traj_;
		};

		/**
		 *  Wrapper for TrajectoryValue to implement the TrajectoryInterface
		 **/
		class TrajectoryValueWrapper : public TrajectoryInterface {
			private:
				const values::TrajectoryValue *trajectoryValue;

			public:
				TrajectoryValueWrapper(const values::TrajectoryValue *t);

			values::PoseValue Get(const TimeStamp &when) const override;
			values::TrajectoryValue::pose_container_t GetAll() const override;
		};

		class TrajectoryOutputInterface : public TrajectoryInterface {
			private:
				const BaseOutput *trajectory_output;
			public:
				TrajectoryOutputInterface(const BaseOutput *t);

				values::PoseValue Get(const TimeStamp &when) const override; 
				values::TrajectoryValue::pose_container_t GetAll() const override;
		};
	}
}

#endif /* TRAJECTORYINTERFACE_H */


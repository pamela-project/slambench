/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */


#ifndef IO_FRAMESOURCE_H
#define IO_FRAMESOURCE_H

#include <cstdint>
#include <map>
#include <vector>

#include <TimeStamp.h>

namespace slambench {
	namespace io {
		class SLAMFrame;
		class Sensor;
		
		class FrameStream {
		public:
			virtual ~FrameStream();
			
			virtual SLAMFrame *GetNextFrame() = 0;
			virtual bool HasNextFrame() = 0;
		};
		
		class FrameCollection {
		public:
			virtual ~FrameCollection();
			
			virtual SLAMFrame *GetFrame(unsigned int index) = 0;
			virtual unsigned int GetFrameCount() = 0;
		};
		
		class FrameCollectionStream : public FrameStream {
		public:
			FrameCollectionStream(FrameCollection &base_collection);
			
			SLAMFrame* GetNextFrame() override;
			bool HasNextFrame() override;
		private:
			FrameCollection &_collection;
			unsigned int _index;
		};
		
		/**
		 * This is a class to separate out GT frames from the actual dataset.
		 * Treated as a regular FrameStream, it will return only dataset frames,
		 * skipping all of the GT frames.
		 */
		class GTBufferingFrameStream : public FrameStream {
		public:
			class GTFrameCollection : public FrameCollection {
			public:
				GTFrameCollection(GTBufferingFrameStream &gt_stream);
				virtual ~GTFrameCollection();
				
				unsigned int GetFrameCount() override;
				SLAMFrame* GetFrame(unsigned int index) override;
				SLAMFrame* GetClosestFrameToTime(slambench::TimeStamp t);
				private:
					GTBufferingFrameStream &gt_stream_;
			};
			
			GTBufferingFrameStream(FrameStream &base_stream);
			virtual ~GTBufferingFrameStream();
			
			SLAMFrame* GetNextFrame() override;
			bool HasNextFrame() override;

			GTFrameCollection *GetGTFrames();
		private:
			void fastForward();
			
			FrameStream &base_stream_;
			SLAMFrame *buffered_frame_;
			std::vector<SLAMFrame*> gt_frames_;
		};
		
		/**
		 * This is a class which returns frames according to how much real
		 * time has passed since the last frame was fetched.
		 * 
		 * If should_pause is set, then the stream will also block until the
		 * frame should actually become available.
		 */
		class RealTimeFrameStream : public FrameStream {
		public:
			RealTimeFrameStream(FrameStream *base_stream, double multiplier, bool should_pause);
			virtual ~RealTimeFrameStream();
			
			SLAMFrame* GetNextFrame() override;
			bool HasNextFrame() override;

		private:
			bool is_valid_frame(SLAMFrame *frame) const;
			
			bool past_first_frame_;
			std::map<slambench::io::Sensor*, slambench::TimeStamp> last_frame_timestamp_;
			std::map<slambench::io::Sensor*, slambench::TimeStamp> last_frame_realtimestamp_;
			
			FrameStream *base_stream_;
			double acceleration_;
			bool should_pause_;
		};
	}
}

#endif /* FRAMESOURCE_H */


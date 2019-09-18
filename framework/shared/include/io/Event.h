/*

 Copyright (c) 2019 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef IO_EVENT_H
#define IO_EVENT_H

#include <TimeStamp.h>

namespace slambench {
  namespace io {
    
    struct Event {
      TimeStamp ts_;
      uint16_t x_;
      uint16_t y_;
      bool polarity_;

      Event(TimeStamp ts, uint16_t x, uint16_t y, bool polarity) : ts_(ts), x_(x), y_(y), polarity_(polarity) {}
    };

    std::ostream& operator << (std::ostream &out, const Event &e) {
      out << e.ts_.S << "." << e.ts_.Ns << " = " << e.x_ << " " << e.y_ << " " << e.polarity_;
      return out;
    }
  }
}

#endif  // IO_EVENT_H

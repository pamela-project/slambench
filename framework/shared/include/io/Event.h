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
      TimeStamp ts;
      uint16_t x;
      uint16_t y;
      bool polarity;

      Event(TimeStamp ts, uint16_t x, uint16_t y, bool polarity) : ts(ts), x(x), y(y), polarity(polarity) {}
    };

    std::ostream& operator << (std::ostream &out, const Event &e) {
      out << e.ts.S << "." << e.ts.Ns << " = " << e.x << " " << e.y << " " << e.polarity;
      return out;
    }
  }
}

#endif  // IO_EVENT_H


#ifndef SLAMBENCHUITYPES_H_
#define SLAMBENCHUITYPES_H_

#include <cstdlib>
#include <cstring>
#include <vector>
#include <iostream>
#include <typeinfo>
#include <mutex>
#include <cassert>

/*
 * ************************************************************************************************
 * ********************** VISIBLE ELEMENTS
 * ************************************************************************************************
 */

struct VisibleElementIdentifier {
    std::string type;
    std::string name;
    VisibleElementIdentifier (std::string t, std::string n) : type(t), name(n) {

    }
};


#endif // SLAMBENCHUITYPES_H_

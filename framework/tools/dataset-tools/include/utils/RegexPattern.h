/*

 Copyright (c) 2017 University of Edinburgh, Imperial College, University of
 Manchester. Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.

 */

#ifndef FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_REGEXPATTERN_H_
#define FRAMEWORK_TOOLS_DATASET_TOOLS_INCLUDE_REGEXPATTERN_H_

class RegexPattern {
 public:
  // characters
  static constexpr auto start = "^";
  static constexpr auto end = "$";

  // components
  static constexpr auto whitespace = "\\s+";
  static constexpr auto integer = "(\\d+)";
  static constexpr auto decimal = "([-0-9.]+)";
  static constexpr auto nanoseconds = "([0-9]+)";
  static constexpr auto number = R"(([+\-]?(?:0|[1-9]\d*)(?:\.\d*)?(?:[eE][+\-]?\d+)?|[-0-9.]+))";
  static constexpr auto timestamp = "([0-9]+)[.]([0-9]+)";
  static constexpr auto filename = "(.*)";
  static constexpr auto lowercase_key = "([a-z_]+)";

  // full patterns
  static constexpr auto comment = "^\\s*#.*$";
};

#endif
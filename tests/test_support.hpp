#pragma once

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#define CHECK_TRUE(expression)                                                   \
  do {                                                                           \
    if (!(expression)) {                                                         \
      throw std::runtime_error(std::string("check failed: ") + #expression);     \
    }                                                                            \
  } while (false)

inline void checkNear(double actual, double expected, double tolerance,
                      const char* label) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(std::string(label) + ": expected " +
                             std::to_string(expected) + ", got " +
                             std::to_string(actual));
  }
}

template <class Function>
int runTest(const char* name, Function&& function) {
  try {
    function();
    std::cout << "PASS " << name << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "FAIL " << name << ": " << error.what() << '\n';
    return 1;
  }
}


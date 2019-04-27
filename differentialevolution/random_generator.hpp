/*
 * Copyright (c) 2017, Adrian Michel
 * http://www.amichel.com
 *
 * This software is released under the 3-Clause BSD License
 *
 * The complete terms can be found in the attached LICENSE file
 * or at https://opensource.org/licenses/BSD-3-Clause
 */

#pragma once

#include <random>
#include <cmath>

namespace amichel {
namespace de {

inline double genrand(double min = 0, double max = 1) {
  static std::mt19937 gen;
  std::uniform_real_distribution<double> dist(min, max);

  return dist(gen);
}

inline int genintrand(double min, double max, bool upperexclusive = false) {
  assert(min < max);
  int ret = 0;
  do
    ret = std::round(genrand(min, max));
  while (ret < min || ret > max || upperexclusive && ret == max);
  return ret;
}
}  // namespace de
}  // namespace amichel

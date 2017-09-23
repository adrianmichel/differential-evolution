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

#include <boost/math/special_functions/round.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include <boost/random/variate_generator.hpp>

namespace amichel {
namespace de {

inline double genrand(double min = 0, double max = 1) {
  static boost::random::mt19937 gen;
  boost::random::uniform_real_distribution<> dist(min, max);
  boost::variate_generator<boost::random::mt19937&,
                           boost::random::uniform_real_distribution<double> >
      value(gen, dist);

  return value();
}

inline int genintrand(double min, double max, bool upperexclusive = false) {
  assert(min < max);
  int ret = 0;
  do
    ret = boost::math::round(genrand(min, max));
  while (ret < min || ret > max || upperexclusive && ret == max);
  return ret;
}
}  // namespace de
}  // namespace amichel

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

#define _USE_MATH_DEFINES 1
#include <math.h>

 /**
  * \defgroup de-console
  * @{
  */

// x^2 - has a min 0
inline double x_sqr_min_function(const amichel::de::DVector& args) {
  double arg(args.at(0));
  double result = pow(arg, 2);
  //		std::cout << "arg: " << arg << ", result = " << result <<
  //std::endl;
  return result;
}

// -(x^2) has a max of 0
inline double x_sqr_max_function(const amichel::de::DVector& args) { return -pow(args.at(0), 2); }

inline double AnotherSimpleFunction(const amichel::de::DVector& args) {
  return pow(args.at(0), 6) - 10 * pow(args.at(0), 3);
}

inline double SphereFunction(const amichel::de::DVector& args) {
    double x1(args.at(0));
    double x2(args.at(1));
    return x1 * x1 + x2 * x2;
}

inline double AckleyFunction(const amichel::de::DVector& args) {
  // return pow( (*args)[ 0 ], 2 );

  double x1(args.at(0));
  double x2(args.at(1));

  double e4 = x1 * x1 + x2 * x2;
  double e0 = sqrt(e4 / 2.0);
  double e1 = exp(-0.2 * e0);
  double e2 = cos(2.0 * M_PI * x1) + cos(2.0 * M_PI * x2);
  double e3 = exp(e2 / 2.0);

  return 20 + M_E - 20 * e1 - e3;
}

inline double SecondDeJongFunction(const amichel::de::DVector& args) {
  double x1(args.at(0));
  double x2(args.at(1));

  return 100.0 * pow((x1 * x1 - x2), 2) + pow(1 - x1, 2);
}

// min: -1.031628453
inline double SixHumpCamelBackFunction(const amichel::de::DVector& args) {
  double x1(args.at(0));
  double x2(args.at(1));

  return (4.0 - 2.1 * x1 * x1 + pow(x1, 4) / 3) * x1 * x1 + x1 * x2 +
    (-4.0 + 4.0 * x2 * x2) * x2 * x2;
}

/**
 * @}
 */

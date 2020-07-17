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

#include <differential_evolution.hpp>
#include "objective_function.h"

 /**
  * \defgroup de-console
  * @{
  */

  // x^2 - has a min 0
class x_sqr_min_function : public objective_function {
public:
  x_sqr_min_function() : objective_function("x sqr min function") {}

  virtual double operator()(amichel::de::DVectorPtr args) {
    double arg((*args)[0]);
    double result = pow(arg, 2);
    //		std::cout << "arg: " << arg << ", result = " << result <<
    //std::endl;
    return result;
  }
};

// -(x^2) has a max of 0
class x_sqr_max_function : public objective_function {
public:
  x_sqr_max_function() : objective_function("- x sqr max function") {}

  virtual double operator()(amichel::de::DVectorPtr args) { return -pow((*args)[0], 2); }
};

class AnotherSimpleFunction : public objective_function {
public:
  AnotherSimpleFunction() : objective_function("a simple function") {}

  virtual double operator()(amichel::de::DVectorPtr args) {
    return pow((*args)[0], 6) - 10 * pow((*args)[0], 3);
  }
};

class SphereFunction : public objective_function {
public:
  SphereFunction() : objective_function("Sphere function") {}

  virtual double operator()(amichel::de::DVectorPtr args) {
    double x1((*args)[0]);
    double x2((*args)[1]);
    return x1 * x1 + x2 * x2;
  }
};

class AckleyFunction : public objective_function {
public:
  AckleyFunction() : objective_function("Auckley function") {}

  virtual double operator()(amichel::de::DVectorPtr args) {
    // return pow( (*args)[ 0 ], 2 );

    double x1((*args)[0]);
    double x2((*args)[1]);

    double e4 = x1 * x1 + x2 * x2;
    double e0 = sqrt(e4 / 2.0);
    double e1 = exp(-0.2 * e0);
    double e2 = cos(2.0 * M_PI * x1) + cos(2.0 * M_PI * x2);
    double e3 = exp(e2 / 2.0);

    return 20 + M_E - 20 * e1 - e3;
  }
};

class SecondDeJongFunction : public objective_function {
public:
  SecondDeJongFunction() : objective_function("Second DeJong function") {}

  virtual double operator()(amichel::de::DVectorPtr args) {
    double x1((*args)[0]);
    double x2((*args)[1]);

    return 100.0 * pow((x1 * x1 - x2), 2) + pow(1 - x1, 2);
  }
};

// min: -1.031628453
class SixHumpCamelBackFunction : public objective_function {
public:
  SixHumpCamelBackFunction()
    : objective_function("Six hump camel back function") {}

  virtual double operator()(amichel::de::DVectorPtr args) {
    double x1((*args)[0]);
    double x2((*args)[1]);

    return (4.0 - 2.1 * x1 * x1 + pow(x1, 4) / 3) * x1 * x1 + x1 * x2 +
      (-4.0 + 4.0 * x2 * x2) * x2 * x2;
  }
};

/**
 * @}
 */

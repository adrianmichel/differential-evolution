/*
 * Copyright (c) 2017, Adrian Michel
 * http://www.amichel.com
 *
 * This software is released under the 3-Clause BSD License
 *
 * The complete terms can be found in the attached LICENSE file
 * or at https://opensource.org/licenses/BSD-3-Clause
 */

#ifndef DE_OBJECTIVE_FUNCTION_HPP_INCLUDED
#define DE_OBJECTIVE_FUNCTION_HPP_INCLUDED

// MS compatible compilers support #pragma once

#if defined(_MSC_VER) && (_MSC_VER >= 1020)
#pragma once
#endif

#include <boost/shared_ptr.hpp>

#include <de_types.hpp>
#include <processors.hpp>

/**
 * Abstract base class for concrete objective functions.
 */
class objective_function {
 private:
  const std::string m_name;

 public:
  /**
   * constructs an objective_function object
   *
   * @param name the objective function name
   */
  objective_function(const std::string& name) : m_name(name) {}

  virtual ~objective_function() {}

  /**
   * Implemented in derived classes - it contains the objective
   * function to optimize.
   *
   * An objective function takes a vector of double arguments and
   * returns the calculated double value.
   *
   * Each index in the args vector corresponds to the same index
   * in the constraints vector. If for example the objective
   * function requires two variables, then set the constraints
   * vector's first two elements to the constraint for the two
   * variables, and in the objective function operator() use the
   * first two values of the args vector as the two variables. All
   * the other values in this vector can be ignored.
   *
   * @param args the vector of arguments. The vector is usually
   *  		   much larger than the number of variables used by
   *  		   the objective function, so the OF will take only
   *  		   the first n values in the vector, and ignore the
   *  		   rest.
   *
   *
   *
   * @return double the function cost which is the value that
   *  	   needs to be optimized
   */
  virtual double operator()(amichel::de::DVectorPtr args) = 0;

  /**
   * An objective function has a name
   *
   * @return const std::string&
   */
  const std::string& name() const { return m_name; }
};

/**
 * Smart pointer to an objective function
 */
using objective_function_ptr = boost::shared_ptr<objective_function>;

#endif  // DE_OBJECTIVE_FUNCTION_HPP_INCLUDED

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

#include <exception>
#include <vector>

namespace amichel {
namespace de {
/**
 * encapsulation of a double type
 *
 * used mostly for debugging and diagnostic purposes
 *
 * @author adrian (12/4/2011)
 */
class Double {
 private:
  double m_value;

 public:
  /**
   * constructs a double initialized with value
   *
   * @author adrian (12/4/2011)
   *
   * @param value
   */
  Double(double value) : m_value(value) {}

  /**
   * default constructor - constructs a Double set to 0
   *
   * @author adrian (12/4/2011)
   */
  Double() : m_value(0) {}

  /**
   * assignment operator, sets the Double to "value"
   *
   * @author adrian (12/4/2011)
   *
   * @param value value to set the Double to
   *
   * @return double returns the current value
   */
  double operator=(double value) {
    m_value = value;

    return m_value;
  }

  /**
   * conversion operator to a double
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  operator double() const { return m_value; }
};

/**
 * a vector of Double instances
 */
typedef std::vector<Double> DVector;

/**
 * shared pointer to a DVector
 */
typedef boost::shared_ptr<DVector> DVectorPtr;

/**
 * de exception conforms to the C++ standard (MS implementation
 * of std::exception has non-standard constructors)
 *
 * @author adrian (4/11/2012)
 */
class exception : public std::exception {
 private:
  const std::string m_what;

 public:
  virtual ~exception() throw() {}
  /**
   * Constructor that takes a C string as argument which is the
   * message associated with the exception
   *
   * @author adrian (4/11/2012)
   *
   * @param what
   */
  exception(const char* what) : m_what(what != 0 ? what : "") {}

  /**
   * To be used with care - if the exception object goes out of
   * scope, the pointer returned by this function becomes invalid
   *
   * @author adrian (4/11/2012)
   *
   * @return const char* the message associated with the exception
   */
  virtual const char* what() const throw() { return m_what.c_str(); }
};
}  // namespace de
}  // namespace amichel

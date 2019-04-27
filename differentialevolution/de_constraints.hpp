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

#include <sstream>
#include <set>
#include <functional>
#include <memory>
#include <cmath>

#include "de_types.hpp"
#include "random_generator.hpp"

namespace amichel {
namespace de {

/**
 * Exception thrown in case of a constraint error
 *
 * @author adrian (12/1/2011)
 */
class constraints_exception : public exception {
 public:
  /**
   * constructor that takes the error message as argument
   *
   * @author adrian (12/4/2011)
   *
   * @param message
   */
  constraints_exception(const std::string& message)
      : exception(message.c_str()) {}
};

/**
 * Abstract base class for concrete constraint classes
 *
 * A constraint class describes certain characteristics and
 * limits of the input variables fed to the objective function.
 *
 * @author adrian (12/1/2011)
 */
class constraint {
 public:
  virtual ~constraint() {}

  /**
   * returns a random value limited to the type and range of the
   * constraint
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  virtual double get_rand_value() = 0;

  /**
   * returns a random value limited to the type and range of the
   * constraint based on a previous value and an origin (see
   * specific implementation in derived classes)
   *
   * @author adrian (12/4/2011)
   *
   * @param value
   * @param origin
   *
   * @return double
   */
  virtual double get_rand_value(double value, double origin) = 0;

  /**
   * returns the min limit of the range
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  virtual double min() const = 0;

  /**
   * returns the max limit of the range
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  virtual double max() const = 0;

  /**
   * Gets a random value within the limits set for the constraint,
   * but further limited to a range defined by its origin and the
   * width of the zone around this origin in pct of the total
   * width
   *
   * @author adrian (12/13/2011)
   *
   * @param origin the origin (center) of the zone further
   *  			 limiting the constraint
   * @param zonePct the width of the zone in pct of the total
   *  			  width, around the origin
   *
   * @return double
   */
  virtual double get_rand_value_in_zone(double origin,
                                        double zonePct) const = 0;

  /**
   * Gets the point midway between min and max - will only work
   * for range constraints
   *
   * @author adrian (12/16/2011)
   *
   * @return double
   */
  virtual double get_middle_point() = 0;
};

/**
 * A smart pointer to a Constraint
 */
typedef std::shared_ptr<constraint> constraint_ptr;

/**
 * Base class for constraints that are range based. Each such
 * constraint has a min and a max value
 *
 * @author adrian (12/1/2011)
 */
class range_constraint : public constraint {
 private:
  double m_min;
  double m_max;

 public:
  /**
   * constructor that takes the min and max limits of the range
   *
   * @author adrian (12/4/2011)
   *
   * @param min
   * @param max
   */
  range_constraint(double min, double max) : m_min(min), m_max(max) {
    assert(min <= max);
  }

  /**
   * returns the min limit of the range
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  double min() const { return m_min; }

  /**
   * returns the max limit of the range
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  double max() const { return m_max; }
};

/**
 * A real constraint. Specifies that variables can have any
 * double value, whitin the specified limits.
 *
 * @author adrian (12/1/2011)
 */
class real_constraint : public range_constraint {
 public:
  /**
   * constructor that takes the min and max limit of the real
   * constraint
   *
   * @author adrian (12/4/2011)
   *
   * @param min
   * @param max
   */
  real_constraint(double min, double max) : range_constraint(min, max) {
    assert(min <= max);
  }

  /**
   * returns a random value limited to the type and range of the
   * constraint
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  double get_rand_value() {
    return genrand(range_constraint::min(), range_constraint::max());
  }

  /**
   * returns a random value limited to the type and range of the
   * constraint based on a previous value and an origin
   *
   * @author adrian (12/4/2011)
   *
   * @param value
   * @param origin
   *
   * @return double
   */
  double get_rand_value(double value, double origin) {
    double ret = value;

    while (ret < range_constraint::min()) {
      ret = range_constraint::min() +
            genrand() * (origin - range_constraint::min());
    }

    while (ret > range_constraint::max()) {
      ret = range_constraint::max() +
            genrand() * (origin - range_constraint::max());
    }

    return ret;
  }

  virtual double get_rand_value_in_zone(double origin, double zonePct) const {
    if (origin > max()) throw constraints_exception("origin coordinate > max");
    if (origin < min()) throw constraints_exception("origin coordinate < min");

    if (zonePct > 100.0) throw constraints_exception("zonePct > 100%");

    if (zonePct < 0) throw constraints_exception("zonePct < 0%");

    if (zonePct == 0) throw constraints_exception("zonePct == 0%");

    double zoneSize = (max() - min()) * zonePct / 100.0;

    double _min = std::max(min(), origin - zoneSize / 2.0);
    double _max = std::min(max(), origin + zoneSize / 2.0);

    return genrand(_min, _max);
  }

  virtual double get_middle_point() { return (max() + min()) / 2.0; }
};

/**
 * An integer constraint. Specifies that variables can have any
 * integer values within the specified limits.
 *
 * @author adrian (12/1/2011)
 */
class int_constraint : public range_constraint {
 public:
  /**
   * constructor that takes the min and max limit of the integer
   * constraint
   *
   * @author adrian (12/4/2011)
   *
   * @param min
   * @param max
   */
  int_constraint(double min, double max) : range_constraint(min, max) {
    assert(min <= max);
  }

  /**
   * returns a random value limited to the type and range of the
   * constraint
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  double get_rand_value() {
    return genintrand(range_constraint::min(), range_constraint::max());
  }

  /**
   * returns a random value limited to the type and range of the
   * constraint based on a previous value and an origin
   *
   * @author adrian (12/4/2011)
   *
   * @param value
   * @param origin
   *
   * @return double
   */
  double get_rand_value(double value, double origin) {
    double ret = std::round(value);

    while (ret < range_constraint::min()) {
      ret = range_constraint::min() +
            genrand() * (origin - range_constraint::min());
      ret = std::round(ret);
    }

    while (ret > range_constraint::max()) {
      ret = range_constraint::max() +
            genrand() * (origin - range_constraint::max());
      ret = std::round(ret);
    }

    return ret;
  }

  virtual double get_rand_value_in_zone(double origin, double zonePct) const {
    if (origin > max()) throw constraints_exception("origin coordinate > max");
    if (origin < min()) throw constraints_exception("origin coordinate < min");

    if (zonePct > 100.0) throw constraints_exception("zonePct > 100%");

    if (zonePct < 0) throw constraints_exception("zonePct < 0%");

    if (zonePct == 0) throw constraints_exception("zonePct == 0%");

    double zoneSize = (max() - min()) * zonePct / 100.0;

    double _min = std::max(min(), origin - zoneSize / 2.0);
    double _max = std::min(max(), origin + zoneSize / 2.0);

    double val = std::round(genrand(_min, _max));

    for (; val < _min || val > _max;
         val = std::round(genrand(_min, _max)))
      ;

    return val;
  }

  virtual double get_middle_point() {
    return std::round((max() - min()) / 2.0);
  }
};

/**
 * A set constraint. Specifies that variables can take any
 * values from a predefined set. Doesn't require min or max.
 *
 * Note that duplicate values will be removed
 *
 * @author adrian (12/1/2011)
 */
class set_constraint : public constraint {
 private:
  class unique : public std::function<bool(Double)> {
   public:
    bool operator()(Double d) const { return m_unique.insert(d).second; }

   public:
    double min() const {
      if (m_unique.size() > 0)
        return *m_unique.begin();
      else
        throw constraints_exception(
            "could not get the min value of an empty set constraint");
    }

    double max() const {
      if (m_unique.size() > 0)
        return *m_unique.rbegin();
      else
        throw constraints_exception(
            "could not get the max value of an empty set constraint");
    }

   private:
    mutable std::set<Double> m_unique;
  };

 private:
  unique m_unique;
  de::DVector m_values;

 public:
  /**
   * Constructs the set from a vector of Double values, and
   * removes duplicates to ensure a uniform distribution for
   * randomization
   *
   * @author adrian (12/10/2011)
   *
   * @param values
   */
  set_constraint(const de::DVector& values) {
    // making sure the values in the "set" are unique
    std::remove_copy_if(values.begin(), values.end(),
                        std::back_inserter(m_values), m_unique);
  }

  /**
   * adds an individual value to the "set", won't create duplicate
   * values.
   *
   * @author adrian (12/10/2011)
   *
   * @param value
   */
  void add_value(de::Double value) {
    if (m_unique(value)) m_values.push_back(value);
  }

  /**
   * returns a value randomly chosen from the set of available
   * values
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  virtual double get_rand_value() {
    de::DVector::size_type index(genintrand(0, m_values.size() - 1));

    return m_values[index];
  }

  /**
   * returns a value randomly chosen from the set of available
   * values
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  virtual double get_rand_value(double value, double origin) {
    return get_rand_value();
  }

  /**
   * returns the min limit of the range
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  double min() const { return m_unique.min(); }

  /**
   * returns the max limit of the range
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  double max() const { return m_unique.max(); }

  virtual double get_rand_value_in_zone(double origin, double zonePct) const {
    throw constraints_exception(
        "get_rand_value_in_zone only supported for range constraints");
  }

  virtual double get_middle_point() {
    throw constraints_exception(
        "get_middle_point not supported by set constraint");
  }
};

/**
 * A boolean constraint. Specifies that variables can take a
 * boolean value - true or false.
 *
 * @author adrian (12/1/2011)
 */
class boolean_constraint : public constraint {
 public:
  /**
   * returns a random boolean value
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  virtual double get_rand_value() { return genrand() < 0.5; }

  /**
   * returns a random boolean value
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  virtual double get_rand_value(double value, double origin) {
    return get_rand_value();
  }

  /**
   * returns the min limit of the range
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  double min() const { return 0; }

  /**
   * returns the max limit of the range
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  double max() const { return 1; }

  virtual double get_rand_value_in_zone(double origin, double zonePct) const {
    throw constraints_exception(
        "get_rand_value_in_zone only supported for range constraints");
  }

  virtual double get_middle_point() {
    throw constraints_exception(
        "get_middle_point not supported by bool constraint");
  }
};

typedef std::vector<constraint_ptr> constraints_base;

/**
 * A collection of constraints, implemented as a vector.
 *
 * Is used to define the constraints for all variables used
 * during an optimization session.
 *
 * @author adrian (12/1/2011)
 */
class constraints : public constraints_base {
 public:
  /**
   * Initializes a collection of constraints with default values.
   * All constraints are of type real and have the same default
   * min and max.
   *
   * @author adrian (12/1/2011)
   *
   * @param varCount number of constraints
   * @param defMin default min limit
   * @param defMax default max limit
   */
  constraints(size_t varCount, double defMin, double defMax)
      : constraints_base(varCount,
                         std::make_shared<real_constraint>(defMin, defMax)) {}

  /**
   * returns a random value limited to the type and range of the
   * constraint
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  double get_rand_value(size_t index) {
    if (index < constraints_base::size())
      return (*this)[index]->get_rand_value();
    else {
      std::stringstream sout;
      sout << "invalid constraint index: " << index << ", higher than max number of constraints: " << constraints_base::size();
      throw constraints_exception(sout.str());
    }
  }

  /**
   * returns a random value limited to the type and range of the
   * constraint based on a previous value and an origin
   *
   * @author adrian (12/4/2011)
   *
   * @param index the constraint index
   * @param value previous value
   * @param origin origin
   *
   * @return double
   */
  double get_rand_value(size_t index, double value, double origin) {
    if (index < constraints_base::size())
      return (*this)[index]->get_rand_value(value, origin);
    else {
      std::stringstream sout;
      sout << "invalid constraint index: " << index << ", higher than max number of constraints: " << constraints_base::size();
      throw constraints_exception(sout.str());
    }
  }

  /**
   * Generates a set of random values within a "hypercube" region
   * defined by an origin and an area expressed as a percentage of
   * the entire range
   *
   * @author adrian (12/13/2011)
   *
   * @param origin the origin coordinates of the region
   * @param sidePct the side of the hypercube in pct of the entire
   *  			  side
   *
   * @return DVectorPtr
   */
  DVectorPtr get_square_zone_rand_values(const DVectorPtr origin,
                                         double sidePct) const {
    assert(origin);
    assert(sidePct > 0 && sidePct <= 100);

    if (origin->size() == constraints_base::size()) {
      DVectorPtr square(std::make_shared<DVector>(origin->size()));

      for (constraints_base::size_type n = 0; n < constraints_base::size(); ++n)
        (*square)[n] =
            (*this)[n]->get_rand_value_in_zone((*origin)[n], sidePct);

      return square;

    } else
      throw constraints_exception(
          "The origin vector must have the same number of elements as there "
          "are constraints");
  }

  DVectorPtr get_middle_point() {
    DVectorPtr r(std::make_shared<DVector>(constraints_base::size()));

    for (constraints_base::size_type n = 0; n < constraints_base::size(); ++n)
      (*r)[n] = (*this)[n]->get_middle_point();

    return r;
  }

  /**
   * Get a set of random values within the limits set by the
   * constraints
   *
   * @author adrian (12/13/2011)
   *
   * @return DVectorPtr
   */
  DVectorPtr get_rand_values() const {
    DVectorPtr r(std::make_shared<DVector>(constraints_base::size()));

    for (constraints_base::size_type n = 0; n < constraints_base::size(); ++n)
      (*r)[n] = (*this)[n]->get_rand_value();

    return r;
  }
};

typedef std::shared_ptr<constraints> constraints_ptr;
}  // namespace de
}  // namespace amichel

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

#include <algorithm>
#include <cassert>
#include <cmath>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <random>
#include <set>
#include <sstream>
#include <thread>
#include <tuple>
#include <vector>

namespace amichel {
namespace de {

//-----------------------------------------------------------------------------
// de_types.hpp

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
typedef std::shared_ptr<DVector> DVectorPtr;

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

//-----------------------------------------------------------------------------
// random_generator.hpp

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

//-----------------------------------------------------------------------------
// de_constraints.hpp

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
   *         limiting the constraint
   * @param zonePct the width of the zone in pct of the total
   *          width, around the origin
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
   *          side
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

//-----------------------------------------------------------------------------
// multithread.hpp

typedef std::mutex mutex;
typedef std::lock_guard<mutex> lock;

//-----------------------------------------------------------------------------
// individual.hpp

class individual;
typedef std::shared_ptr<individual> individual_ptr;

/**
 * An individual of a Differrential Evolution population
 *
 * An individual has a set of variables and a cost associated
 * with these variables
 *
 * An individual is thread safe
 *
 * @author adrian (12/1/2011)
 */
class individual {
 private:
  de::DVectorPtr m_vars;
  double m_cost;
  de::mutex m_mx;

 public:
  /**
   * constructs an individual
   *
   * @author adrian (12/4/2011)
   *
   * @param varCount the number of variables for each individual
   */
  individual(size_t varCount)
      : m_vars(std::make_shared<de::DVector>(varCount)) {}

  /**
   * constructs an individual
   *
   * @author adrian (12/4/2011)
   *
   * @param vars a vector of variables that will be copied into
   *         the internal vector of variables
   */
  individual(const de::DVector& vars)
      : m_vars(std::make_shared<de::DVector>(vars)) {}

  /**
   * Initialized the internal vector of variables with random
   * values within the constraints
   *
   * @author adrian (12/4/2011)
   *
   * @param constraints
   */
  void init(constraints_ptr constraints) {
    assert(constraints);
    assert(m_vars);
    assert(m_vars->size() == constraints->size());

    for (de::DVector::size_type j = 0; j < m_vars->size(); ++j)
      (*m_vars)[j] = constraints->get_rand_value(j);
  }

  /**
   * returns the cost
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  double cost() const { return m_cost; }

  /**
   * Sets the variables to new random values within the
   * constraints, using origin and old value
   *
   * @author adrian (12/4/2011)
   *
   * @param constraints
   * @param origin
   */
  void ensureConstraints(constraints_ptr constraints, de::DVectorPtr origin) {
    assert(constraints);
    assert(m_vars);
    assert(origin);
    assert(m_vars->size() == constraints->size());

    for (de::DVector::size_type j = 0; j < m_vars->size(); ++j) {
      (*m_vars)[j] = constraints->get_rand_value(j, (*m_vars)[j], (*origin)[j]);
    }
  }

  /**
   * returns the internal variable vector
   *
   * @author adrian (12/4/2011)
   *
   * @return de::DVectorPtr
   */
  de::DVectorPtr vars() const { return m_vars; }

  /**
   * returns a non constant reference to a variable value
   * based on an index, which can be used as an lvalue.
   *
   * @author adrian (12/4/2011)
   *
   * @param index the index of the variable whose reference to
   *        return
   *
   * @return de::Double&
   */
  de::Double& operator[](size_t index) { return (*m_vars)[index]; }

  /**
   * returns a constant reference to a variable value based
   * on an index, which can be used as an lvalue.
   *
   * @author adrian (12/4/2011)
   *
   * @param index the index of the variable whose reference to return
   *
   * @return de::Double&
   */
  const de::Double& operator[](size_t index) const { return (*m_vars)[index]; }

  /**
   * Sets the cost
   *
   * @author adrian (12/4/2011)
   *
   * @param cost
   */
  void setCost(double cost) { m_cost = cost; }

  /**
   * compares the current individual with another individual based
   * on cost. Returns true if the current cost is lower or equal
   * to the cost of the other individual
   *
   * @author adrian (12/4/2011)
   *
   * @param ind the individual to compare with
   *
   * @return bool
   */
  bool operator<=(const individual& ind) const {
    assert(ind.size() == size());
    return cost() <= ind.cost();
  }

  /**
   * compares the current individual with another individual based
   * on cost. Returns true if the current cost is strictly lower
   * than the cost of the other individual
   *
   * @author adrian (12/4/2011)
   *
   * @param ind the individual to compare with
   *
   * @return bool
   */
  bool operator<(const individual& ind) const {
    assert(ind.size() == size());
    return cost() < ind.cost();
  }

  /**
   * compares the current individual with another individual based
   * on cost. Returns true if the current cost is better or equal
   * than the cost of the other individual, where better is either
   * lower or higher depending on the minimize flag
   *
   * @author adrian (12/4/2011)
   *
   * @param ind the individual to compare with
   * @param minimize <= if true, >= if false
   *
   * @return bool
   */
  bool better_or_equal(const individual_ptr ind, bool minimize) const {
    assert(ind);
    return minimize ? *this <= *ind : *ind <= *this;
  }

  /**
   * compares the current individual with another individual based
   * on cost. Returns true if the current cost is strictly better
   * than the cost of the other individual, where better is either
   * lower or higher depending on the minimize flag
   *
   * @author adrian (12/4/2011)
   *
   * @param ind the individual to compare with
   * @param minimize < if true, > if false
   *
   * @return bool
   */
  bool better(const individual_ptr ind, bool minimize) const {
    assert(ind);
    return minimize ? *this < *ind : *ind < *this;
  }

  /**
   * returns the size of variables vector
   *
   * @author adrian (12/4/2011)
   *
   * @return size_t
   */
  size_t size() const { return m_vars->size(); }

  /**
   * returns a string representation of the internals of an
   * individual (cost and list of variables)
   *
   * @author adrian (12/4/2011)
   *
   * @return std::string
   */
  std::string to_string() const {
    std::ostringstream os;

    os << "cost: " << cost() << ", vars: ";

    for (de::DVector::size_type j = 0; j < m_vars->size(); ++j) {
      os << (*m_vars)[j] << ", ";
    }

    return os.str();
  }
};

typedef std::queue<individual_ptr> individual_queue_base;

/**
 * A thread safe queue of individuals
 *
 * Used to queue Individuals containing the arguments to be
 * passed by different processors to the objective function
 *
 * @author adrian (12/1/2011)
 */
class individual_queue : public individual_queue_base {
 private:
  de::mutex m_mx;

 public:
  /**
   * adds a new individual at the bottom of the queue
   *
   * is thread safe
   *
   * @author adrian (12/4/2011)
   *
   * @param ind the individual to insert into the queue
   */
  void push_back(individual_ptr ind) {
    de::lock lock(m_mx);

    individual_queue_base::push(ind);
  }

  /**
   * removes the individual from the top of the queue (if any) and
   * returns it
   *
   * is thread safe
   *
   * @author adrian (12/4/2011)
   *
   * @return individual_ptr the individual at the top of the queue
   *       or null individual if the queue is empty
   */
  individual_ptr pop() {
    de::lock lock(m_mx);

    if (!individual_queue_base::empty()) {
      individual_ptr p(individual_queue_base::front());

      individual_queue_base::pop();

      return p;
    } else
      return individual_ptr();
  }
};

//-----------------------------------------------------------------------------
// listener.hpp

/**
 * Abstract base class for user defined listeners.
 *
 * A Listener class will receive notifications when certain
 * significan events happen during an optimization session
 *
 * These events can be used to dispaly the current status,
 * diagnostic information, etc
 *
 * * The user will derive concrete listener classes from this
 * class, which will do something useful with the received
 * events
 *
 * @author adrian (12/1/2011)
 */
class listener {
 public:
  virtual ~listener() {}

  /**
   * called at the start of the optimization process
   *
   * @author adrian (12/4/2011)
   */
  virtual void start() = 0;
  /**
   * called at the end of the optimization process
   *
   * this function is called even if the optimization process
   * ends with an exception
   *
   * @author adrian (12/4/2011)
   */
  virtual void end() = 0;
  /**
   * called if an exception was thrown during the optimization
   * process, and signals an error
   *
   * @author adrian (12/4/2011)
   */
  virtual void error() = 0;
  /**
   * called at the start of each generation
   *
   * @author adrian (12/4/2011)
   *
   * @param genCount
   */
  virtual void startGeneration(size_t genCount) = 0;
  /**
   * called at the end of each generation, unless an exception is
   * thrown before reaching the end of the iteration
   *
   * @author adrian (12/4/2011)
   *
   * @param genCount
   * @param bestIndGen
   * @param bestInd
   */
  virtual void endGeneration(size_t genCount, individual_ptr bestIndGen,
                             individual_ptr bestInd) = 0;
  /**
   * called before the selection starts
   *
   * @author adrian (12/4/2011)
   *
   * @param genCount
   */
  virtual void startSelection(size_t genCount) = 0;
  /**
   * called after the selection has been performed
   *
   * @author adrian (12/4/2011)
   *
   * @param genCount
   */
  virtual void endSelection(size_t genCount) = 0;
  /**
   * called before starting the objective function processing
   *
   * @author adrian (12/4/2011)
   *
   * @param genCount
   */
  virtual void startProcessors(size_t genCount) = 0;
  /**
   * called after the objective function processing has been
   * completed
   *
   * @author adrian (12/4/2011)
   *
   * @param genCount
   */
  virtual void endProcessors(size_t genCount) = 0;
};

/**
 * A smart pointer to a Listener
 */
typedef std::shared_ptr<listener> listener_ptr;

/**
 * A concrete Listener that ignores all received events
 *
 * @author adrian (12/1/2011)
 */
class null_listener : public listener {
 public:
  virtual void start() {}
  virtual void end() {}
  virtual void error() {}
  virtual void startGeneration(size_t genCount) {}
  virtual void endGeneration(size_t genCount, individual_ptr bestIndGen,
                             individual_ptr bestInd) {}
  virtual void startSelection(size_t genCount) {}
  virtual void endSelection(size_t genCount) {}
  virtual void startProcessors(size_t genCount) {}
  virtual void endProcessors(size_t genCount) {}
};

//-----------------------------------------------------------------------------
// population.hpp

typedef std::vector<individual_ptr> population_base;

/**
 * A collection of individuals.
 *
 * @author adrian (12/4/2011)
 */
class population : public population_base {
 public:
  /**
   * constructs a population object containing uninitialized
   * individuals
   *
   * @author adrian (12/4/2011)
   *
   * @param popSize the population size (number of individuals)
   * @param varCount the number of variables for each individual
   */
  population(size_t popSize, size_t varCount) : population_base(popSize) {
    assert(popSize > 0);
    assert(varCount > 0);

    init(popSize, varCount);
  }

  /**
   * constructs a population object and initializes its
   * individuals by first setting all variables to random values
   * within the limits imposed by the corresponding constraints,
   * then runs the objective function for each individual to
   * calculate the associated cost.
   *
   * @author adrian (12/4/2011)
   *
   * @param popSize the population size (number of individuals)
   * @param varCount the number of variables for each individual
   * @param constraints constraints to use when initializing the
   *            individuals
   */
  population(size_t popSize, size_t varCount, constraints_ptr constraints)
      : population_base(popSize) {
    assert(popSize > 0);
    assert(varCount > 0);

    init(popSize, varCount);

    for (population::size_type i = 0; i < size(); ++i) at(i)->init(constraints);
  }

  /**
   * returns the best individual in a population
   *
   * @author adrian (12/4/2011)
   *
   * @return individual_ptr
   */
  individual_ptr best(bool minimize) const {
    population::size_type best(0);

    for (population::size_type i = 0; i < size(); ++i)
      best = at(i)->better_or_equal(at(best), minimize) ? i : best;

    return at(best);
  }

  /**
   * returns the string representation of a population
   *
   * @author adrian (12/4/2011)
   *
   * @return std::string string representation of the population
   */
  std::string to_string() {
    std::string str;
    for (population::size_type i = 0; i < size(); ++i)
      str += at(i)->to_string();

    return str;
  }

 private:
  void init(size_t popSize, size_t varCount) {
    for (population_base::size_type i = 0; i < size(); ++i)
      operator[](i) = std::make_shared<individual>(varCount);
  }

 public:
};

typedef std::shared_ptr<population> population_ptr;

//-----------------------------------------------------------------------------
// mutation_strategy.hpp

#define URN_DEPTH 5

/**
 * Parameters used by mutation strategies
 * weight factor, crossover factor and dither factor which is
 * calculated from the previous two
 *
 * @author adrian (12/1/2011)
 */
class mutation_strategy_arguments {
 private:
  const double m_weight;
  const double m_crossover;
  const double m_dither;

 public:
  /**
   * constructs a mutation_strategy_arguments object.
   *
   * Besides the weight and crossover factors, which are supplied
   * by the calling code, this object holds a dither factor, which
   * is calculated upon construction, and is used by some mutation
   * strtegies.
   *
   * @author adrian (12/4/2011)
   *
   * @param weight weight factor which is a double between 0-2 as
   *         defined by the differential evolution algorithm
   * @param crossover crossover factor which is a double between
   *          0-1 as defined by the differential evolution
   *          algorithm
   */
  mutation_strategy_arguments(double weight, double crossover)
      : m_weight(weight),
        m_crossover(crossover),
        m_dither(weight + genrand() * (1.0 - weight)) {
    // todo: test or assert that weight and crossover are within bounds (0-1,
    // 0-2 or something)
  }

  /**
   * returns the weight factor
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  double weight() const { return m_weight; }
  /**
   * returns the crossover factor
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  double crossover() const { return m_crossover; }
  /**
   * returns the dither factor
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  double dither() const { return m_dither; }
};

/**
 * A an abstract based class for mutation strategies
 *
 * A mutation strategy defines how varaibles values are adjusted
 * during the optimization process
 *
 * @author adrian (12/1/2011)
 */
class mutation_strategy {
 private:
  mutation_strategy_arguments m_args;
  size_t m_varCount;

 protected:
  /**
   * Used to generate a set of 4 random size_t numbers
   * by the mutation strategy as indexes
   *
   * The numbers must all be different, and also different from an
   * index supplied externally
   *
   * @author adrian (12/1/2011)
   */
  class Urn {
    size_t m_urn[URN_DEPTH];

   public:
    /**
     * Constructs an urn object
     *
     * @author adrian (12/4/2011)
     *
     * @param NP upper limit (exclusive) for the generated random
     *       numbers
     * @param avoid value to avoid when generating the random
     *        numbers
     */
    Urn(size_t NP, size_t avoid) {
      do
        m_urn[0] = genintrand(0, NP, true);
      while (m_urn[0] == avoid);
      do
        m_urn[1] = genintrand(0, NP, true);
      while (m_urn[1] == m_urn[0] || m_urn[1] == avoid);
      do
        m_urn[2] = genintrand(0, NP, true);
      while (m_urn[2] == m_urn[1] || m_urn[2] == m_urn[0] || m_urn[2] == avoid);
      do
        m_urn[3] = genintrand(0, NP, true);
      while (m_urn[3] == m_urn[2] || m_urn[3] == m_urn[1] ||
             m_urn[3] == m_urn[0] || m_urn[3] == avoid);
    }

    /**
     * returns one of the four generated random numbers
     *
     * @author adrian (12/4/2011)
     *
     * @param index the index of the random number to return, can be
     *        between 0-3
     *
     * @return size_t
     */
    size_t operator[](size_t index) const {
      assert(index < 4);
      return m_urn[index];
    }
  };

 public:
  virtual ~mutation_strategy() {}

  /**
   * constructs a mutation strategy
   *
   * @author adrian (12/4/2011)
   *
   * @param varCount number of variables
   * @param args mutation strategy arguments
   */
  mutation_strategy(size_t varCount, const mutation_strategy_arguments& args)
      : m_args(args), m_varCount(varCount) {}

  /**
   * type for the tuple returned by the operator() member.
   */
  typedef std::tuple<individual_ptr, de::DVectorPtr> mutation_info;

  /**
   * performs the mutation
   *
   * @author adrian (12/4/2011)
   *
   * @param pop a reference to the current population
   * @param bestIt the best individual of the previous generation
   * @param i the current individual index
   *
   * @return mutation_info tuple containing the mutated individual
   *       and a vector of doubles of the same size as the
   *       number of variables, used as origin to generate new
   *       values in case they exceed the limits imposed by the
   *       corresponding constraints
   */
  virtual mutation_info operator()(const population& pop, individual_ptr bestIt,
                                   size_t i) = 0;

  /**
   * returns the number of variables
   *
   * @author adrian (12/4/2011)
   *
   * @return size_t
   */
  size_t varCount() const { return m_varCount; }

  /**
   * returns the weight factor
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  double weight() const { return m_args.weight(); }

  /**
   * returns the crossover factor
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  double crossover() const { return m_args.crossover(); }

  /**
   * returns the dither factor
   *
   * @author adrian (12/4/2011)
   *
   * @return double
   */
  double dither() const { return m_args.dither(); }
};

typedef std::shared_ptr<mutation_strategy> mutation_strategy_ptr;

/**
 * Mutation strategy #1
 *
 * @author adrian (12/1/2011)
 */
class mutation_strategy_1 : public mutation_strategy {
 public:
  /**
   * constructs a mutation strategy # 1
   *
   * @author adrian (12/4/2011)
   *
   * @param varCount number of variables
   * @param args mutation strategy arguments
   */
  mutation_strategy_1(size_t varCount, const mutation_strategy_arguments& args)
      : mutation_strategy(varCount, args) {}

  /**
   * performs the mutation
   *
   * @author adrian (12/4/2011)
   *
   * @param pop a reference to the current population
   * @param bestIt the best individual of the previous generation
   * @param i the current individual index
   *
   * @return mutation_info tuple containing the mutated individual
   *       and a vector of doubles of the same size as the
   *       number of variables, used as origin to generate new
   *       values in case they exceed the limits imposed by the
   *       corresponding constraints
   */
  mutation_info operator()(const population& pop, individual_ptr bestIt,
                           size_t i) {
    assert(bestIt);

    de::DVectorPtr origin(std::make_shared<de::DVector>(varCount()));
    individual_ptr tmpInd(std::make_shared<individual>(*pop[i]->vars()));
    Urn urn(pop.size(), i);

    // make sure j is within bounds
    size_t j = genintrand(0, varCount(), true);
    size_t k = 0;

    do {
      (*tmpInd->vars())[j] =
          (*pop[urn[0]]->vars())[j] +
          weight() * ((*pop[urn[1]]->vars())[j] - (*pop[urn[2]]->vars())[j]);

      j = ++j % varCount();
      ++k;
    } while (genrand() < crossover() && k < varCount());

    origin = pop[urn[0]]->vars();

    return mutation_info(tmpInd, origin);
  }
};

/**
 * mutation strategy # 2
 *
 * @author adrian (12/4/2011)
 */
class mutation_strategy_2 : public mutation_strategy {
 public:
  /**
   * constructs a mutation strategy # 2
   *
   * @author adrian (12/4/2011)
   *
   * @param varCount number of variables
   * @param args mutation strategy arguments
   */
  mutation_strategy_2(size_t varCount, const mutation_strategy_arguments& args)
      : mutation_strategy(varCount, args) {}

  /**
   * performs the mutation
   *
   * @author adrian (12/4/2011)
   *
   * @param pop a reference to the current population
   * @param bestIt the best individual of the previous generation
   * @param i the current individual index
   *
   * @return mutation_info tuple containing the mutated individual
   *       and a vector of doubles of the same size as the
   *       number of variables, used as origin to generate new
   *       values in case they exceed the limits imposed by the
   *       corresponding constraints
   */
  mutation_info operator()(const population& pop, individual_ptr bestIt,
                           size_t i) {
    assert(bestIt);

    de::DVectorPtr origin(std::make_shared<de::DVector>(varCount()));
    individual_ptr tmpInd(std::make_shared<individual>(*pop[i]->vars()));
    Urn urn(pop.size(), i);

    // make sure j is within bounds
    size_t j = genintrand(0, varCount(), true);
    size_t k = 0;

    do {
      (*tmpInd->vars())[j] =
          (*tmpInd->vars())[j] +
          weight() * ((*bestIt->vars())[j] - (*tmpInd->vars())[j]) +
          weight() * ((*pop[urn[1]]->vars())[j] - (*pop[urn[2]]->vars())[j]);

      j = ++j % varCount();
      ++k;
    } while (genrand() < crossover() && k < varCount());

    origin = pop[urn[0]]->vars();

    return mutation_info(tmpInd, origin);
  }
};

/**
 * mutation strategy # 3
 *
 * @author adrian (12/4/2011)
 */
class mutation_strategy_3 : public mutation_strategy {
 public:
  /**
   * constructs a mutation strategy # 3
   *
   * @author adrian (12/4/2011)
   *
   * @param varCount number of variables
   * @param args mutation strategy arguments
   */
  mutation_strategy_3(size_t varCount, const mutation_strategy_arguments& args)
      : mutation_strategy(varCount, args) {}

  /**
   * performs the mutation
   *
   * @author adrian (12/4/2011)
   *
   * @param pop a reference to the current population
   * @param bestIt the best individual of the previous generation
   * @param i the current individual index
   *
   * @return mutation_info tuple containing the mutated individual
   *       and a vector of doubles of the same size as the
   *       number of variables, used as origin to generate new
   *       values in case they exceed the limits imposed by the
   *       corresponding constraints
   */
  mutation_info operator()(const population& pop, individual_ptr bestIt,
                           size_t i) {
    assert(bestIt);

    de::DVectorPtr origin(std::make_shared<de::DVector>(varCount()));
    individual_ptr tmpInd(std::make_shared<individual>(*pop[i]->vars()));
    Urn urn(pop.size(), i);

    // make sure j is within bounds
    size_t j = genintrand(0, varCount(), true);
    size_t k = 0;

    do {
      double jitter = (0.0001 * genrand() + weight());

      (*tmpInd->vars())[j] =
          (*bestIt->vars())[j] +
          jitter * ((*pop[urn[1]]->vars())[j] - (*pop[urn[2]]->vars())[j]);

      j = ++j % varCount();
      ++k;
    } while (genrand() < crossover() && k < varCount());

    origin = pop[urn[0]]->vars();

    return mutation_info(tmpInd, origin);
  }
};

/**
 * mutation strategy # 4
 *
 * @author adrian (12/4/2011)
 */
class mutation_strategy_4 : public mutation_strategy {
 public:
  /**
   * constructs a mutation strategy # 4
   *
   * @author adrian (12/4/2011)
   *
   * @param varCount number of variables
   * @param args mutation strategy arguments
   */
  mutation_strategy_4(size_t varCount, const mutation_strategy_arguments& args)
      : mutation_strategy(varCount, args) {}

  /**
   * performs the mutation
   *
   * @author adrian (12/4/2011)
   *
   * @param pop a reference to the current population
   * @param bestIt the best individual of the previous generation
   * @param i the current individual index
   *
   * @return mutation_info tuple containing the mutated individual
   *       and a vector of doubles of the same size as the
   *       number of variables, used as origin to generate new
   *       values in case they exceed the limits imposed by the
   *       corresponding constraints
   */
  mutation_info operator()(const population& pop, individual_ptr bestIt,
                           size_t i) {
    assert(bestIt);

    de::DVectorPtr origin(std::make_shared<de::DVector>(varCount()));
    individual_ptr tmpInd(std::make_shared<individual>(*pop[i]->vars()));
    Urn urn(pop.size(), i);

    // make sure j is within bounds
    size_t j = genintrand(0, varCount(), true);
    size_t k = 0;

    double factor(weight() + genrand() * (1.0 - weight()));

    do {
      double jitter = (0.0001 * genrand() + weight());

      (*tmpInd->vars())[j] =
          (*pop[urn[0]]->vars())[j] +
          factor * ((*pop[urn[1]]->vars())[j] - (*pop[urn[2]]->vars())[j]);

      j = ++j % varCount();
      ++k;
    } while (genrand() < crossover() && k < varCount());

    origin = pop[urn[0]]->vars();

    return mutation_info(tmpInd, origin);
  }
};

/**
 * Mutation strategy # 5
 *
 * @author adrian (12/1/2011)
 */
class mutation_strategy_5 : public mutation_strategy {
 public:
  /**
   * constructs a mutation strategy # 5
   *
   * @author adrian (12/4/2011)
   *
   * @param varCount number of variables
   * @param args mutation strategy arguments
   */
  mutation_strategy_5(size_t varCount, const mutation_strategy_arguments& args)
      : mutation_strategy(varCount, args) {}

  /**
   * performs the mutation
   *
   * @author adrian (12/4/2011)
   *
   * @param pop a reference to the current population
   * @param bestIt the best individual of the previous generation
   * @param i the current individual index
   *
   * @return mutation_info tuple containing the mutated individual
   *       and a vector of doubles of the same size as the
   *       number of variables, used as origin to generate new
   *       values in case they exceed the limits imposed by the
   *       corresponding constraints
   */
  mutation_info operator()(const population& pop, individual_ptr bestIt,
                           size_t i) {
    assert(bestIt);

    de::DVectorPtr origin(std::make_shared<de::DVector>(varCount()));
    individual_ptr tmpInd(std::make_shared<individual>(*pop[i]->vars()));
    Urn urn(pop.size(), i);

    // make sure j is within bounds
    size_t j = genintrand(0, varCount(), true);
    size_t k = 0;

    do {
      (*tmpInd->vars())[j] =
          (*pop[urn[0]]->vars())[j] +
          dither() * ((*pop[urn[1]]->vars())[j] - (*pop[urn[2]]->vars())[j]);

      j = ++j % varCount();
      ++k;
    } while (genrand() < crossover() && k < varCount());

    origin = pop[urn[0]]->vars();
    return mutation_info(tmpInd, origin);
  }
};

//-----------------------------------------------------------------------------
// processors.hpp

/**
 * Abstract based class for Processor listeners that receive
 * events from processors.
 *
 * Since its methods are called from multiple threads,
 * concrete classes must use thread synchronization objects to
 * avoid data corruption
 *
 * @author adrian (12/1/2011)
 */
class processor_listener {
 public:
  virtual ~processor_listener() {}
  /**
   * called at the start of a processor operator() which runs the
   * objective function
   *
   * @author adrian (12/4/2011)
   *
   * @param index the processor index
   */
  virtual void start(size_t index) = 0;
  /**
   * called before running the objective function with variables
   * from the current individual
   *
   * @author adrian (12/4/2011)
   *
   * @param index the processor index
   * @param individual current individual that the objective
   *           function runs on
   */
  virtual void start_of(size_t index, individual_ptr individual) = 0;
  /**
   * called after running the objective function with variables
   * from the current individual. The indvidual passed as argument
   * also has the cost set to the result of the objective function
   * run
   *
   * @author adrian (12/4/2011)
   *
   * @param index the processor index
   * @param individual current individual that the objective
   *           function ran on. Also contains the cost
   */
  virtual void end_of(size_t index, individual_ptr individual) = 0;
  /**
   * called at the end of a processor operator() which runs the
   * objective function
   *
   * this is called even if an exception is thrown
   *
   * @author adrian (12/4/2011)
   *
   * @param index the processor index
   */
  virtual void end(size_t index) = 0;
  /**
   * called if an exception is thrown during the run of the
   * objective function, and indicates an error
   *
   * @author adrian (12/4/2011)
   *
   * @param index the processor index
   * @param message a message describing the error
   */
  virtual void error(size_t index, const std::string& message) = 0;
};

/**
 * basic implementation of a processor_listener that doesn't do
 * anything.
 *
 * Doesn't need synchronization, since no data is read or
 * modified
 *
 * @author adrian (12/4/2011)
 */
class null_processor_listener : public processor_listener {
 public:
  /**
   * called at the start of a processor operator() which runs the
   * objective function
   *
   * @author adrian (12/4/2011)
   *
   * @param index the processor index
   */
  virtual void start(size_t index) {}
  /**
   * called before running the objective function with variables
   * from the current individual
   *
   * @author adrian (12/4/2011)
   *
   * @param index the processor index
   * @param individual current individual that the objective
   *           function runs on
   */
  virtual void start_of(size_t index, individual_ptr individual) {}
  /**
   * called after running the objective function with variables
   * from the current individual. The indvidual passed as argument
   * also has the cost set to the result of the objective function
   * run
   *
   * @author adrian (12/4/2011)
   *
   * @param index the processor index
   * @param individual current individual that the objective
   *           function ran on. Also contains the cost
   */
  virtual void end_of(size_t index, individual_ptr individual) {}
  /**
   * called at the end of a processor operator() which runs the
   * objective function
   *
   * this is called even if an exception is thrown
   *
   * @author adrian (12/4/2011)
   *
   * @param index the processor index
   */
  virtual void end(size_t index) {}
  /**
   * called if an exception is thrown during the run of the
   * objective function, and indicates an error
   *
   * @author adrian (12/4/2011)
   *
   * @param index the processor index
   * @param message an message describing the error
   */
  virtual void error(size_t index, const std::string& message) {}
};

/**
 * A pointer to a processor listener
 */
typedef std::shared_ptr<processor_listener> processor_listener_ptr;

/**
 * Exception thrown in case of an error in the objective
 * function.
 *
 * @author adrian (12/1/2011)
 */
class objective_function_exception : public exception {
 public:
  /**
   * constructs an objectivr_function_exception object
   *
   * @author adrian (12/11/2011)
   *
   * @param message the message describing the error that caused
   *          the exception
   */
  objective_function_exception(const std::string& message)
      : exception(message.c_str()) {}
};

/**
 * Interface to an objective function factory. If the objective
 * function requires that a different instance be passed to each
 * processor, create a concrete objective_function_factory
 * derived from this class and implement the virtual make method
 * to create a new instance of the objective function.
 *
 * Use a reference or shared_ptr to an
 * objective_function_exception as processors template argument,
 * and pass the corresponding object as constructor argument.
 *
 * The corresponding processor_traits class above will ensure
 * that the right behavior is applied.
 *
 * The template argument T is the type of the objective function
 *
 * @author adrian (12/15/2011)
 */
template <typename T>
class objective_function_factory {
 public:
  /**
   * Defines a type pointer to an objective function
   */
  typedef std::shared_ptr<T> T_ptr;

  /**
   * virtual distructor
   *
   * @author adrian (12/15/2011)
   */
  virtual ~objective_function_factory() {}

  /**
   * Method implemented in derived classes that will create new
   * instances of the objective function
   *
   * @author adrian (12/15/2011)
   *
   * @return T_ptr a smart pointer to the objective function
   */
  virtual T_ptr make() = 0;
};

/**
 * Base processor traits for the case where the objecive
 * function is passed by reference and copied
 *
 * @author adrian (12/15/2011)
 */
template <typename T>
class processor_traits {
 public:
  // \cond
  typedef T value_type;
  static double run(T t, de::DVectorPtr vars) { return t(vars); }
  static T make(T t) { return t; }
  // \endcond
};

/**
 * specialized processor traits for the case where the objective
 * function is passed as a ponter
 *
 * @author adrian (12/15/2011)
 */
template <typename T>
class processor_traits<T*> {
 public:
  // \cond
  typedef T* value_type;
  static double run(value_type t, de::DVectorPtr vars) { return (*t)(vars); }
  static value_type make(value_type t) { return t; }
  // \endcond
};

/**
 * Specialized processor traits for the case where the objective
 * function is passed as a shared pointer
 *
 * @author adrian (12/15/2011)
 */
template <typename T>
class processor_traits<std::shared_ptr<T> > {
 public:
  // \cond
  typedef std::shared_ptr<T> value_type;
  static double run(value_type t, de::DVectorPtr vars) { return (*t)(vars); }
  static value_type make(value_type t) { return t; }
  // \endcond
};

/**
 * Specialized processor traits for the case wehere the
 * processor receives a pointer to an objective function
 * factory.
 *
 * @author adrian (12/15/2011)
 */
template <typename T>
class processor_traits<objective_function_factory<T>*> {
 public:
  // \cond
  typedef std::shared_ptr<T> value_type;
  static double run(value_type t, de::DVectorPtr vars) { return (*t)(vars); }
  static value_type make(objective_function_factory<T>* off) {
    return off->make();
  }
  // \endcond
};
/**
 * Specialized processor traits for the case wehere the
 * processor receives a shared pointer to an objective function
 * factory.
 *
 * @author adrian (12/15/2011)
 */
template <typename T>
class processor_traits<std::shared_ptr<objective_function_factory<T> > > {
 public:
  // \cond
  typedef std::shared_ptr<T> value_type;
  static double run(value_type t, de::DVectorPtr vars) { return (*t)(vars); }
  static value_type make(
      std::shared_ptr<objective_function_factory<T> > off) {
    return off->make();
  }
  // \endcond
};

/**
 * Specialized processor traits for the case wehere the
 * processor receives a reference to an objective
 * function factory.
 *
 * @author adrian (12/15/2011)
 */
template <typename T>
class processor_traits<objective_function_factory<T>&> {
 public:
  // \cond
  typedef std::shared_ptr<T> value_type;
  static double run(value_type t, de::DVectorPtr vars) { return (*t)(vars); }
  static value_type make(objective_function_factory<T>& off) {
    return off.make();
  }
  // \endcond
};

/**
 * A processor runs the objective function in one thread. There
 * can be any number of processors running the objective
 * function in parallel in as many threads.
 *
 * The processor class uses the type of the objective function
 * defined in the corresponding processor_traits
 *
 * @author adrian (12/1/2011)
 */
template <typename T>
class processor {
 private:
  typename processor_traits<T>::value_type m_of;
  individual_queue& m_indQueue;
  processor_listener_ptr m_listener;
  size_t m_index;

  bool m_result;

 public:
  /**
   * constructs a processor object
   *
   * @author adrian (12/4/2011)
   *
   * @param index the processor index
   * @param of objective function, or objective function factory.
   *       Accepts pointer, shared pointer, reference
   * @param indQueue queue containing the individuals to process
   * @param listener listener that will receive notifications of
   *           important events during the processing of the
   *           objective function
   */
  processor(size_t index, T of, individual_queue& indQueue,
            processor_listener_ptr listener)
      : m_of(processor_traits<T>::make(of)),
        m_indQueue(indQueue),
        m_result(false),
        m_listener(listener),
        m_index(index) {
    assert(listener);
  }

  // Instead of write class processor : boost::noncopyable
  processor(const processor&) = delete;
  processor& operator=(const processor&) = delete;

  /**
   * runs the objective function on the object at the top of the
   * queue, if any
   *
   * @author adrian (12/4/2011)
   */
  void operator()() {
    m_listener->start(m_index);
    m_result = false;
    try {
      for (individual_ptr ind = m_indQueue.pop(); ind; ind = m_indQueue.pop()) {
        m_listener->start_of(m_index, ind);
        double result = processor_traits<T>::run(m_of, ind->vars());

        ind->setCost(result);
        m_listener->end_of(m_index, ind);
      }
      m_result = true;

    m_listener->end(m_index);
    } catch (const objective_function_exception& e) {
      m_result = false;
      m_listener->error(m_index, e.what());
    }
  }

  /**
   * indicates whether the run ended succesfully when the thread
   * exits
   *
   * @author adrian (12/4/2011)
   *
   * @return bool
   */
  bool success() const { return m_result; }
};

/**
 * Exception thrown in case of a processors error
 *
 * @author adrian (12/1/2011)
 */
class processors_exception : exception {
 public:
  /**
   * constructor taking a message string as argument
   *
   * @author adrian (12/15/2011)
   *
   * @param message
   */
  processors_exception(const std::string& message)
      : exception(message.c_str()) {}
};

/**
 * A collection of processors
 *
 * This class starts and coordinates the various processors
 * during an optimization session.
 *
 * Takes the type of the objective function or objective
 * function factory as argument (reference, pointer or
 * shared_ptr)
 *
 * @author adrian (12/1/2011)
 */
template <typename T>
class processors {
 private:
  typedef std::shared_ptr<std::vector<std::thread> > thread_group_ptr;
  typedef std::shared_ptr<processor<T> > processor_ptr;
  typedef std::vector<processor_ptr> processor_vector;
  typedef std::shared_ptr<T> T_ptr;

 private:
  individual_queue m_indQueue;
  processor_vector m_processors;
  thread_group_ptr m_threads;

 public:
  /**
   * constructs a processors object, which in turn constructs the
   * "count" processors, using the objective_function provided
   *
   * @author adrian (12/4/2011)
   *
   * @param count number of processors to create
   * @param of objective function or objective function factory
   * @param listener a listener passed to each created processor
   */
  processors(size_t count, T of, processor_listener_ptr listener) {
    assert(count > 0);
    assert(listener);

    for (size_t n = 0; n < count; ++n) {
      processor_ptr processor(std::make_shared<processor<T> >(
          n, of, std::ref(m_indQueue), listener));
      m_processors.push_back(processors<T>::processor_ptr(processor));
    }
  }

  /**
   * pushes on individual to the bottom of the processing queue
   *
   * @author adrian (12/4/2011)
   *
   * @param ind
   */
  void push(individual_ptr ind) { m_indQueue.push(ind); }
  /**
   * starts all processors threads asynchronously (it will not
   * wait for them to finish)
   *
   * @author adrian (12/4/2011)
   */
  void start() {
    // create a new group every time, don't bother removing all individual
    // threads
    m_threads = std::make_shared<std::vector<std::thread>>();

    for (typename processor_vector::size_type n = 0; n < m_processors.size();
         ++n) {
      processor_ptr p(m_processors[n]);
      m_threads->emplace_back(std::ref(*p));
    }
  }

  /**
   * waits for all processors to finish before returning
   *
   * used for synchronous processing
   *
   * @author adrian (12/4/2011)
   */
  void wait() {
    for (auto& i : *m_threads)
      i.join();

    if (!m_indQueue.empty())
      throw processors_exception("threads ended before emptying the queue");

    if (!success()) throw processors_exception("objective function error");
  }

  /**
   * indicates whether all processors ended succesfully
   *
   * @author adrian (12/4/2011)
   *
   * @return bool true if success, false if an error occured
   */
  bool success() {
    for (typename processor_vector::size_type n = 0; n < m_processors.size();
         ++n) {
      processor_ptr processor(m_processors[n]);
      if (!processor->success()) return false;
    }

    return true;
  }

  /**
   * pushes all individuals in a population into the processing
   * queue
   *
   * @author adrian (12/15/2011)
   *
   * @param population
   */
  void push(population_ptr population) {
    std::copy(population->begin(), population->end(),
              std::back_inserter(m_indQueue));
  }

  /**
   * A smart pointer to a collection of processors
   */
  typedef std::shared_ptr<processors<T> > processors_ptr;
};

//-----------------------------------------------------------------------------
// selection_strategy.hpp

/**
 * Abstract based class defining the interface for a selection
 * strategy.
 *
 * A selection strategy is used by Differential Evolution to
 * determine what is the best individual
 *
 * @author adrian (12/1/2011)
 */
class selection_strategy {
 public:
  virtual ~selection_strategy() {}

  /**
   * applies the selection strategy
   *
   * @author adrian (12/4/2011)
   *
   * @param pop1 old population
   * @param pop2 new population
   * @param bestInd reference to the best individual - contains
   *          the best individual on return
   * @param minimize if true, it will minimize, if false it will
   *           maximize
   */
  virtual void operator()(population_ptr& pop1, population_ptr& pop2,
                          individual_ptr& bestInd, bool minimize) = 0;
};

/**
 * A smart pointer to a selection strategy
 */
typedef std::shared_ptr<selection_strategy> selection_strategy_ptr;

/**
 * Selection strategy that sorts all individuals across two
 * generations based on the cost for each individual, and on the
 * desired outcome - minimization or maximization of the
 * objective function
 *
 * @author adrian (12/1/2011)
 */
class best_parent_child_selection_strategy : public selection_strategy {
 public:
  /**
   * applies the selection strategy
   *
   * @author adrian (12/4/2011)
   *
   * @param pop1 old population
   * @param pop2 new population
   * @param bestInd reference to the best individual - contains
   *          the best individual on return
   * @param minimize if true, it will minimize, if false it will
   *           maximize
   */
  void operator()(population_ptr& pop1, population_ptr& pop2,
                  individual_ptr& bestInd, bool minimize) {
    assert(pop1);
    assert(pop2);

    assert(pop1->size() == pop2->size());

    sort_across(*pop1, *pop2, minimize);

    // this is the best
    bestInd = (*pop1)[0];
  }

 private:
  class individual_compare {
   private:
    const bool m_minimize;

   public:
    individual_compare(bool minimize) : m_minimize(minimize) {}

    bool operator()(individual_ptr ind1, individual_ptr ind2) {
      assert(ind1 && ind2);

      return ind1->better(ind2, m_minimize);
    }
  };

  void sort_across(population& v1, population& v2, bool minimize) {
    v1.insert(v1.end(), v2.begin(), v2.end());
    v2.clear();

    std::sort(v1.begin(), v1.end(), individual_compare(minimize));

    v2.insert(v2.end(), v1.begin() + v1.size() / 2, v1.end());

    v1.erase(v1.begin() + v1.size() / 2, v1.end());
  }
};

/**
 * Selection strategy which compares individuals in two
 * generations corresponding to the same index, and uses the
 * best one for the next generation
 *
 * @author adrian (12/1/2011)
 */
class tournament_selection_strategy : public selection_strategy {
 public:
  /**
   * applies the selection strategy
   *
   * @author adrian (12/4/2011)
   *
   * @param pop1 old population
   * @param pop2 new population
   * @param bestInd reference to the best individual - contains
   *          the best individual on return
   * @param minimize if true, it will minimize, if false it will
   *           maximize
   */
  void operator()(population_ptr& pop1, population_ptr& pop2,
                  individual_ptr& bestInd, bool minimize) {
    assert(pop1);
    assert(pop2);

    assert(pop1->size() == pop2->size());

    for (size_t i = 0; i < pop1->size(); ++i) {
      individual_ptr crt((*pop2)[i]);

      if (crt->better_or_equal((*pop1)[i], minimize)) {
        if (crt->better_or_equal(bestInd, minimize)) bestInd = crt;
      } else
        (*pop2)[i] = (*pop1)[i];
    }

    std::swap(pop1, pop2);
  }
};

//-----------------------------------------------------------------------------
// termination_strategy.hpp

/**
 * Abstract base class defining the interface used by a concrete
 * Termination Strategy class
 *
 * A termination strategy defines the logic used to stop the
 * optimization process. Can be as simple as a generation
 * counter that exits when the max number of generations has
 * been reached, or can implement more complex algorithms that
 * try to minimize the number of execution steps by
 * determinining when a reasonable best value has been reached.
 *
 * @author adrian (12/1/2011)
 */
class termination_strategy {
 public:
  virtual ~termination_strategy() {}

  /**
   *
   *
   * @author adrian (12/1/2011)
   *
   * @param best The best individual so far
   * @param genCount generation number
   *
   * @return bool return true to continue, or false to stop the
   *       optimization process
   */
  virtual bool event(individual_ptr best, size_t genCount) = 0;
};

/**
 * A smart pointer to a TerminationStrategy
 */
typedef std::shared_ptr<termination_strategy> termination_strategy_ptr;

/**
 * Basic implementation of a Termination Strategy: stop the
 * optimization process if a maximum number of generations has
 * been reached
 *
 * @author adrian (12/1/2011)
 */
class max_gen_termination_strategy : public termination_strategy {
 private:
  const size_t m_maxGen;

 public:
  /**
   * constructs a max_gen_termination_strategy object
   *
   * @author adrian (12/4/2011)
   *
   * @param maxGen maximum number of generations after which the
   *         optimization stops
   */
  max_gen_termination_strategy(size_t maxGen) : m_maxGen(maxGen) {}

  virtual bool event(individual_ptr best, size_t genCount) {
    return genCount < m_maxGen;
  }
};

//-----------------------------------------------------------------------------

/**
 * Exception thrown in case of an error during an optimization
 * session
 *
 * @author adrian (12/1/2011)
 */
class differential_evolution_exception {};

/**
 * Differential evolution main class
 *
 * Runs an optimization session based on various input
 * parameters or strategies
 *
 * @author adrian (12/1/2011)
 */
template <typename T>
class differential_evolution {
 private:
  const size_t m_varCount;
  const size_t m_popSize;

  population_ptr m_pop1;
  population_ptr m_pop2;
  individual_ptr m_bestInd;

  constraints_ptr m_constraints;
  typename processors<T>::processors_ptr m_processors;
  termination_strategy_ptr m_terminationStrategy;
  selection_strategy_ptr m_selectionStrategy;
  mutation_strategy_ptr m_mutationStrategy;
  listener_ptr m_listener;

  const bool m_minimize;

 public:
  /**
   * constructs a differential_evolution object
   *
   * @author adrian (12/4/2011)
   *
   * @param varCount total number of variables. It includes the
   *  			   variables required by the objective function
   *  			   but has many more elements as required by the
   *  			   algorithm
   * @param popSize total number of individuals in a population
   * @param processors number of parallel processors used
   *  				 during an optimization session
   * @param constraints a vector of constraints that contains the
   *  				  constraints for the variables used by the
   *  				  objective function as well as constraints
   *  				  for all other variables used internally by
   *  				  the algorithm
   * @param minimize will attempt to minimize the cost if true, or
   *  			   maximize the cost if false
   * @param terminationStrategy a termination strategy
   * @param selectionStrategy a selection strategy
   * @param mutationStrategy a mutation strategy
   * @param listener a listener
   */
  differential_evolution(size_t varCount, size_t popSize,
                         typename processors<T>::processors_ptr processors,
                         constraints_ptr constraints, bool minimize,
                         termination_strategy_ptr terminationStrategy,
                         selection_strategy_ptr selectionStrategy,
                         mutation_strategy_ptr mutationStrategy,
                         de::listener_ptr listener) try

      : m_varCount(varCount),
        m_popSize(popSize),
        m_pop1(std::make_shared<population>(popSize, varCount, constraints)),
        m_pop2(std::make_shared<population>(popSize, varCount)),
        m_bestInd(m_pop1->best(minimize)),
        m_constraints(constraints),
        m_processors(processors),
        m_minimize(minimize),
        m_terminationStrategy(terminationStrategy),
        m_listener(listener),
        m_selectionStrategy(selectionStrategy),
        m_mutationStrategy(mutationStrategy) {
    assert(processors);
    assert(constraints);
    assert(terminationStrategy);
    assert(selectionStrategy);
    assert(listener);
    assert(mutationStrategy);

    assert(popSize > 0);
    assert(varCount > 0);

    // initializing population 1 by running all objective functions with
    // the initial random arguments
    processors->push(m_pop1);
    processors->start();
    processors->wait();

  } catch (const processors_exception&) {
    throw differential_evolution_exception();
  }

  virtual ~differential_evolution(void) {}

  /**
   * starts a differential evolution optimization process
   *
   * although the processing is done in parallel, this function is
   * synchronous and won't return until the optimization is
   * complete, or an error triggered an exception
   *
   * @author adrian (12/4/2011)
   */
  void run() {
    try {
      m_listener->start();
      individual_ptr bestIndIteration(m_bestInd);

      for (size_t genCount = 0;
           m_terminationStrategy->event(m_bestInd, genCount); ++genCount) {
        m_listener->startGeneration(genCount);
        for (size_t i = 0; i < m_popSize; ++i) {
          mutation_strategy::mutation_info mutationInfo(
              (*m_mutationStrategy)(*m_pop1, bestIndIteration, i));

          individual_ptr tmpInd(std::get<0>(mutationInfo));

          tmpInd->ensureConstraints(m_constraints,
                                    std::get<1>(mutationInfo));

          // populate the queue
          m_processors->push(tmpInd);

          // put temps in a temp vector for now (they are empty until
          // processed), will be moved to the right place after processed
          (*m_pop2)[i] = tmpInd;
        }

        m_listener->startProcessors(genCount);
        m_processors->start();
        m_processors->wait();
        m_listener->endProcessors(genCount);

        // BestParentChildSelectionStrategy()( m_pop1, m_pop2, m_bestInd,
        // m_minimize );
        m_listener->startSelection(genCount);
        (*m_selectionStrategy)(m_pop1, m_pop2, m_bestInd, m_minimize);
        bestIndIteration = m_bestInd;

        m_listener->endSelection(genCount);

        m_listener->endGeneration(genCount, bestIndIteration, m_bestInd);
      }

      m_listener->end();
    } catch (const processors_exception&) {
      m_listener->error();
      throw differential_evolution_exception();
    }
  }

  /**
   * returns the best individual resulted from the optimization
   * process
   *
   * @author adrian (12/4/2011)
   *
   * @return individual_ptr
   */
  individual_ptr best() const { return m_bestInd; }
};

}  // namespace de
}  // namespace amichel
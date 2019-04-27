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

#include <queue>
#include <sstream>

#include "de_constraints.hpp"
#include "de_types.hpp"
#include "multithread.hpp"

namespace amichel {
namespace de {

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
   *  		   the internal vector of variables
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
   *  			return
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
   *  	   or null individual if the queue is empty
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

}  // namespace de
}  // namespace amichel
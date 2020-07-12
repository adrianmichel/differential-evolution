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

#include <boost/tuple/tuple.hpp>
#include "population.hpp"

#define URN_DEPTH 5

namespace amichel {
namespace de {

/**
 * Parameters used by mutation strategies
 * weight factor, crossover factor and dither factor which is
 * calculated from the previous two
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
   * @param weight weight factor which is a double between 0-2 as
   *  			 defined by the differential evolution algorithm
   * @param crossover crossover factor which is a double between
   *  				0-1 as defined by the differential evolution
   *  				algorithm
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
   * @return double
   */
  double weight() const { return m_weight; }
  /**
   * returns the crossover factor
   *
   * @return double
   */
  double crossover() const { return m_crossover; }
  /**
   * returns the dither factor
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
   */
  class Urn {
    size_t m_urn[URN_DEPTH];

   public:
    /**
     * Constructs an urn object
     *
     * @param NP upper limit (exclusive) for the generated random
     *  		 numbers
     * @param avoid value to avoid when generating the random
     *  			numbers
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
     * @param index the index of the random number to return, can be between 0-3
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
   * @param varCount number of variables
   * @param args mutation strategy arguments
   */
  mutation_strategy(size_t varCount, const mutation_strategy_arguments& args)
      : m_args(args), m_varCount(varCount) {}

  /**
   * type for the tuple returned by the operator() member.
   */
  using mutation_info = boost::tuple<individual_ptr, de::DVectorPtr>;

  /**
   * performs the mutation
   *
   * @param pop a reference to the current population
   * @param bestIt the best individual of the previous generation
   * @param i the current individual index
   *
   * @return mutation_info tuple containing the mutated individual
   *  	   and a vector of doubles of the same size as the
   *  	   number of variables, used as origin to generate new
   *  	   values in case they exceed the limits imposed by the
   *  	   corresponding constraints
   */
  virtual mutation_info operator()(const population& pop, individual_ptr bestIt,
                                   size_t i) = 0;

  size_t varCount() const { return m_varCount; }
  double weight() const { return m_args.weight(); }
  double crossover() const { return m_args.crossover(); }
  double dither() const { return m_args.dither(); }
};

using mutation_strategy_ptr = std::shared_ptr<mutation_strategy>;

/**
 * Mutation strategy #1
 */
class mutation_strategy_1 : public mutation_strategy {
 public:
  /**
   * constructs a mutation strategy # 1
   *
   * @param varCount number of variables
   * @param args mutation strategy arguments
   */
  mutation_strategy_1(size_t varCount, const mutation_strategy_arguments& args)
      : mutation_strategy(varCount, args) {}

  /**
   * performs the mutation
   *
   * @param pop a reference to the current population
   * @param bestIt the best individual of the previous generation
   * @param i the current individual index
   *
   * @return mutation_info tuple containing the mutated individual
   *  	   and a vector of doubles of the same size as the
   *  	   number of variables, used as origin to generate new
   *  	   values in case they exceed the limits imposed by the
   *  	   corresponding constraints
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

class mutation_strategy_2 : public mutation_strategy {
 public:
  /**
   * constructs a mutation strategy # 2
   *
   * @param varCount number of variables
   * @param args mutation strategy arguments
   */
  mutation_strategy_2(size_t varCount, const mutation_strategy_arguments& args)
      : mutation_strategy(varCount, args) {}

  /**
   * performs the mutation
   *
   * @param pop a reference to the current population
   * @param bestIt the best individual of the previous generation
   * @param i the current individual index
   *
   * @return mutation_info tuple containing the mutated individual
   *  	   and a vector of doubles of the same size as the
   *  	   number of variables, used as origin to generate new
   *  	   values in case they exceed the limits imposed by the
   *  	   corresponding constraints
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

class mutation_strategy_3 : public mutation_strategy {
 public:
  /**
   * constructs a mutation strategy # 3
   *
   * @param varCount number of variables
   * @param args mutation strategy arguments
   */
  mutation_strategy_3(size_t varCount, const mutation_strategy_arguments& args)
      : mutation_strategy(varCount, args) {}

  /**
   * performs the mutation
   *
   * @param pop a reference to the current population
   * @param bestIt the best individual of the previous generation
   * @param i the current individual index
   *
   * @return mutation_info tuple containing the mutated individual
   *  	   and a vector of doubles of the same size as the
   *  	   number of variables, used as origin to generate new
   *  	   values in case they exceed the limits imposed by the
   *  	   corresponding constraints
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

class mutation_strategy_4 : public mutation_strategy {
 public:
  /**
   * constructs a mutation strategy # 4
   *
   * @param varCount number of variables
   * @param args mutation strategy arguments
   */
  mutation_strategy_4(size_t varCount, const mutation_strategy_arguments& args)
      : mutation_strategy(varCount, args) {}

  /**
   * performs the mutation
   *
   * @param pop a reference to the current population
   * @param bestIt the best individual of the previous generation
   * @param i the current individual index
   *
   * @return mutation_info tuple containing the mutated individual
   *  	   and a vector of doubles of the same size as the
   *  	   number of variables, used as origin to generate new
   *  	   values in case they exceed the limits imposed by the
   *  	   corresponding constraints
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

class mutation_strategy_5 : public mutation_strategy {
 public:
  /**
   * constructs a mutation strategy # 5
   *
   * @param varCount number of variables
   * @param args mutation strategy arguments
   */
  mutation_strategy_5(size_t varCount, const mutation_strategy_arguments& args)
      : mutation_strategy(varCount, args) {}

  /**
   * performs the mutation
   *
   * @param pop a reference to the current population
   * @param bestIt the best individual of the previous generation
   * @param i the current individual index
   *
   * @return mutation_info tuple containing the mutated individual
   *  	   and a vector of doubles of the same size as the
   *  	   number of variables, used as origin to generate new
   *  	   values in case they exceed the limits imposed by the
   *  	   corresponding constraints
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

}  // namespace de
}  // namespace amichel

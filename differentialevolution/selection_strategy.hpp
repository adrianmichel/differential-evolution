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

#include "population.hpp"

namespace amichel {
namespace de {

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
   *  			  the best individual on return
   * @param minimize if true, it will minimize, if false it will
   *  			   maximize
   */
  virtual void operator()(population_ptr& pop1, population_ptr& pop2,
                          individual_ptr& bestInd, bool minimize) = 0;
};

/**
 * A smart pointer to a selection strategy
 */
typedef boost::shared_ptr<selection_strategy> selection_strategy_ptr;

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
   *  			  the best individual on return
   * @param minimize if true, it will minimize, if false it will
   *  			   maximize
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
   *  			  the best individual on return
   * @param minimize if true, it will minimize, if false it will
   *  			   maximize
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

}  // namespace de
}  // namespace amichel
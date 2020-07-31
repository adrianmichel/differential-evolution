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

#include <boost/scope_exit.hpp>
#include <memory>
#include <boost/thread.hpp>
#include <boost/utility.hpp>
#include <queue>

#include "individual.hpp"
#include "population.hpp"

namespace amichel {
namespace de {

/**
 * Abstract based class for Processor listeners that receive
 * events from processors.
 *
 * Since its methods are called from multiple threads,
 * concrete classes must use thread synchronization objects to
 * avoid data corruption
 */
class processor_listener {
 public:
  virtual ~processor_listener() {}
  /**
   * called at the start of a processor operator() which runs the
   * objective function
   *
   * @param index the processor index
   */
  virtual void start(size_t index) = 0;
  /**
   * called before running the objective function with variables
   * from the current individual
   *
   * @param index the processor index
   * @param individual current individual that the objective
   *  				 function runs on
   */
  virtual void start_of(size_t index, individual_ptr individual) = 0;
  /**
   * called after running the objective function with variables
   * from the current individual. The indvidual passed as argument
   * also has the cost set to the result of the objective function
   * run
   *
   * @param index the processor index
   * @param individual current individual that the objective
   *  				 function ran on. Also contains the cost
   */
  virtual void end_of(size_t index, individual_ptr individual) = 0;
  /**
   * called at the end of a processor operator() which runs the
   * objective function
   *
   * this is called even if an exception is thrown
   *
   * @param index the processor index
   */
  virtual void end(size_t index) = 0;
  /**
   * called if an exception is thrown during the run of the
   * objective function, and indicates an error
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
 */
class null_processor_listener : public processor_listener {
 public:
  /**
   * called at the start of a processor operator() which runs the
   * objective function
   *
   * @param index the processor index
   */
  virtual void start(size_t index) {}
  /**
   * called before running the objective function with variables
   * from the current individual
   *
   * @param index the processor index
   * @param individual current individual that the objective
   *  				 function runs on
   */
  virtual void start_of(size_t index, individual_ptr individual) {}
  /**
   * called after running the objective function with variables
   * from the current individual. The indvidual passed as argument
   * also has the cost set to the result of the objective function
   * run
   *
   * @param index the processor index
   * @param individual current individual that the objective
   *  				 function ran on. Also contains the cost
   */
  virtual void end_of(size_t index, individual_ptr individual) {}
  /**
   * called at the end of a processor operator() which runs the
   * objective function
   *
   * this is called even if an exception is thrown
   *
   * @param index the processor index
   */
  virtual void end(size_t index) {}
  /**
   * called if an exception is thrown during the run of the
   * objective function, and indicates an error
   *
   * @param index the processor index
   * @param message an message describing the error
   */
  virtual void error(size_t index, const std::string& message) {}
};

/**
 * A pointer to a processor listener
 */
using processor_listener_ptr = std::shared_ptr<processor_listener>;

/**
 * Exception thrown in case of an error in the objective
 * function.
 */
class objective_function_exception : public exception {
 public:
  /**
   * constructs an objectivr_function_exception object
   *
   * @param message the message describing the error that caused
   *  			  the exception
   */
  objective_function_exception(const std::string& message)
      : exception(message.c_str()) {}
};

using ObjectiveFunction = std::function<double(const de::DVector&)>;

/**
 * A processor runs the objective function in one thread. There
 * can be any number of processors running the objective
 * function in parallel in as many threads.
 *
 * The processor class uses the type of the objective function
 * defined in the corresponding processor_traits
 */
class processor : boost::noncopyable {
 private:
  ObjectiveFunction m_of;
  individual_queue& m_indQueue;
  processor_listener_ptr m_listener;
  size_t m_index;

  bool m_result;

 public:
  /**
   * constructs a processor object
   *
   * @param index the processor index
   * @param of objective function, or objective function factory.
   *  		 Accepts pointer, shared pointer, reference
   * @param indQueue queue containing the individuals to process
   * @param listener listener that will receive notifications of
   *  			   important events during the processing of the
   *  			   objective function
   */
  processor(size_t index, ObjectiveFunction of, individual_queue& indQueue,
            processor_listener_ptr listener)
      : m_of(of),
        m_indQueue(indQueue),
        m_result(false),
        m_listener(listener),
        m_index(index) {
    assert(listener);
  }

  /**
   * runs the objective function on the object at the top of the
   * queue, if any
   */
  void operator()() {
    m_listener->start(m_index);
    m_result = false;
    try {
      for (individual_ptr ind = m_indQueue.pop(); ind; ind = m_indQueue.pop()) {
        m_listener->start_of(m_index, ind);
        double result = std::invoke(m_of, ind->vars());

        ind->setCost(result);
        m_listener->end_of(m_index, ind);
      }
      m_result = true;

      BOOST_SCOPE_EXIT_TPL((&m_index)(&m_listener)) {
        m_listener->end(m_index);
      }
      BOOST_SCOPE_EXIT_END
    } catch (const objective_function_exception& e) {
      m_result = false;
      m_listener->error(m_index, e.what());
    }
  }

  /**
   * indicates whether the run ended succesfully when the thread
   * exits
   *
   * @return bool
   */
  bool success() const { return m_result; }
};

/**
 * Exception thrown in case of a processors error
 */
class processors_exception : exception {
 public:
  /**
   * constructor taking a message string as argument
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
 */
class processors {
 private:
  using thread_group_ptr = std::shared_ptr<boost::thread_group>;
  using processor_ptr = std::shared_ptr<processor>;
  using processor_vector = std::vector<processor_ptr>;

 private:
  individual_queue m_indQueue;
  processor_vector m_processors;
  thread_group_ptr m_threads;

 public:
  /**
   * constructs a processors object, which in turn constructs the
   * "count" processors, using the objective_function provided
   *
   * @param count number of processors to create
   * @param of objective function or objective function factory
   * @param listener a listener passed to each created processor
   */
  processors(size_t count, ObjectiveFunction of, processor_listener_ptr listener) {
    assert(count > 0);
    assert(listener);

    for (size_t n = 0; n < count; ++n) {
      processor_ptr processor(std::make_shared<processor >(
          n, of, boost::ref(m_indQueue), listener));
      m_processors.push_back(processors::processor_ptr(processor));
    }
  }

  /**
   * pushes on individual to the bottom of the processing queue
   *
   * @param ind
   */
  void push(individual_ptr ind) { m_indQueue.push(ind); }
  /**
   * starts all processors threads asynchronously (it will not
   * wait for them to finish)
   */
  void start() {
    // create a new group every time, don't bother removing all individual
    // threads
    m_threads = std::make_shared<boost::thread_group>();

    for (processor_ptr processor : m_processors){
      boost::thread* th(new boost::thread(boost::ref(*processor)));
      m_threads->add_thread(th);
    }
  }

  /**
   * waits for all processors to finish before returning
   *
   * used for synchronous processing
   */
  void wait() {
    m_threads->join_all();

    if (!m_indQueue.empty()) {
      throw processors_exception("threads ended before emptying the queue");
    }

    if (!success()) {
      throw processors_exception("objective function error");
    }
  }

  /**
   * indicates whether all processors ended succesfully
   *
   * @return bool true if success, false if an error occured
   */
  bool success() {
    for (processor_ptr processor : m_processors) {
      if (!processor->success()) {
        return false;
      }
    }

    return true;
  }

  /**
   * pushes all individuals in a population into the processing
   * queue
   *
   * @param population
   */
  void push(population_ptr population) {
    std::copy(population->begin(), population->end(), std::back_inserter(m_indQueue));
  }

  /**
   * A smart pointer to a collection of processors
   */
  using processors_ptr = std::shared_ptr<processors>;
};

}  // namespace de
}  // namespace amichel

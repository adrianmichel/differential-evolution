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
#include <boost/shared_ptr.hpp>
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
   *  				 function runs on
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
   *  				 function ran on. Also contains the cost
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
   *  				 function runs on
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
   *  				 function ran on. Also contains the cost
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
typedef boost::shared_ptr<processor_listener> processor_listener_ptr;

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
   *  			  the exception
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
  typedef boost::shared_ptr<T> T_ptr;

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
class processor_traits<boost::shared_ptr<T> > {
 public:
  // \cond
  typedef boost::shared_ptr<T> value_type;
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
  typedef boost::shared_ptr<T> value_type;
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
class processor_traits<boost::shared_ptr<objective_function_factory<T> > > {
 public:
  // \cond
  typedef boost::shared_ptr<T> value_type;
  static double run(value_type t, de::DVectorPtr vars) { return (*t)(vars); }
  static value_type make(
      boost::shared_ptr<objective_function_factory<T> > off) {
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
  typedef boost::shared_ptr<T> value_type;
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
class processor : boost::noncopyable {
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
   *  		 Accepts pointer, shared pointer, reference
   * @param indQueue queue containing the individuals to process
   * @param listener listener that will receive notifications of
   *  			   important events during the processing of the
   *  			   objective function
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
  typedef boost::shared_ptr<boost::thread_group> thread_group_ptr;
  typedef boost::shared_ptr<processor<T> > processor_ptr;
  typedef std::vector<processor_ptr> processor_vector;
  typedef boost::shared_ptr<T> T_ptr;

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
      processor_ptr processor(boost::make_shared<processor<T> >(
          n, of, boost::ref(m_indQueue), listener));
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
    m_threads = boost::make_shared<boost::thread_group>();

    for (typename processor_vector::size_type n = 0; n < m_processors.size();
         ++n) {
      processor_ptr p(m_processors[n]);
      boost::thread* th(new boost::thread(boost::ref(*p)));
      m_threads->add_thread(th);
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
    m_threads->join_all();

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
  typedef boost::shared_ptr<processors<T> > processors_ptr;
};

}  // namespace de
}  // namespace amichel

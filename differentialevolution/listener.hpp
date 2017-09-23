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

namespace amichel {
namespace de {

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
typedef boost::shared_ptr<listener> listener_ptr;

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

}  // namespace de
}  // namespace amichel
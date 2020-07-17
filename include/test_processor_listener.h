#pragma once

#include <processors.hpp>

/**
 * Very basic processor listener that doesn't do anything.
 *
 * It shows however how to setup a thread safe listener, by
 * using synchronization objects.
 */
class DETestProcessorListener : public amichel::de::processor_listener {
  amichel::de::mutex m_mx;

public:
  virtual void start(size_t index) { amichel::de::lock lock(m_mx); }

  virtual void start_of(size_t index, amichel::de::individual_ptr ind) {
    amichel::de::lock lock(m_mx);
  }

  virtual void end_of(size_t index, amichel::de::individual_ptr ind) { amichel::de::lock lock(m_mx); }

  virtual void end(size_t index) { amichel::de::lock lock(m_mx); }

  virtual void error(size_t index, const std::string& message) {
    amichel::de::lock lock(m_mx);
  }
};

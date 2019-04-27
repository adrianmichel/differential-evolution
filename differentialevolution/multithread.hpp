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

#include <thread>
#include <mutex>

namespace amichel {
namespace de {
typedef std::mutex mutex;
typedef std::lock_guard<mutex> lock;

}  // namespace de
}  // namespace amichel

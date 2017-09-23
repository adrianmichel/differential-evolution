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

#include <boost/thread.hpp>

namespace amichel {
namespace de {
typedef boost::recursive_mutex mutex;
typedef boost::lock_guard<boost::recursive_mutex> lock;

}  // namespace de
}  // namespace amichel

/*
 * This code is a modified version of OkapiLib's units library, which in turn is a modified version of
 * Benjamin Jurke's work in 2015. You can read his blog post
 * here:
 * https://benjaminjurke.com/content/articles/2015/compile-time-numerical-unit-dimension-checking/
 *
 * You can find the OkapiLib code this was taken from here:
 * https://github.com/purduesigbots/OkapiLib
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
#pragma once

#include "rev/api/units/q_angle.hh"
#include "rev/api/units/q_frequency.hh"
#include "rev/api/units/q_time.hh"
#include "rev/api/units/r_quantity.hh"

namespace rev {
QUANTITY_TYPE(0, 0, -1, 1, QAngularSpeed)

constexpr QAngularSpeed radps = radian / second;
constexpr QAngularSpeed rpm = (360 * degree) / minute;
constexpr QAngularSpeed cps = (0.01 * degree) / second; // centidegree per second

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
static QAngularSpeed convertHertzToRadPerSec(QFrequency in) {
  return (in.convert(Hz) / 2_pi) * radps;
}
#pragma GCC diagnostic pop

inline namespace literals {
constexpr QAngularSpeed operator"" _rpm(long double x) {
  return x * rpm;
}
constexpr QAngularSpeed operator"" _rpm(unsigned long long int x) {
  return static_cast<double>(x) * rpm;
}
} // namespace literals
} // namespace rev

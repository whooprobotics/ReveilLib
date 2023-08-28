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

#include "rev/api/units/q_acceleration.hh"
#include "rev/api/units/q_mass.hh"
#include "rev/api/units/r_quantity.hh"

namespace rev {
QUANTITY_TYPE(1, 1, -2, 0, QForce)

constexpr QForce newton = (kg * meter) / (second * second);
constexpr QForce poundforce = pound * G;
constexpr QForce kilopond = kg * G;

inline namespace literals {
constexpr QForce operator"" _n(long double x) {
  return QForce(x);
}
constexpr QForce operator"" _n(unsigned long long int x) {
  return QForce(static_cast<double>(x));
}
constexpr QForce operator"" _lbf(long double x) {
  return static_cast<double>(x) * poundforce;
}
constexpr QForce operator"" _lbf(unsigned long long int x) {
  return static_cast<double>(x) * poundforce;
}
constexpr QForce operator"" _kp(long double x) {
  return static_cast<double>(x) * kilopond;
}
constexpr QForce operator"" _kp(unsigned long long int x) {
  return static_cast<double>(x) * kilopond;
}
} // namespace literals
} // namespace rev

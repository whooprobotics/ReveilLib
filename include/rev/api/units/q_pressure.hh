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
#include "rev/api/units/q_area.hh"
#include "rev/api/units/q_mass.hh"
#include "rev/api/units/r_quantity.hh"

namespace rev {
QUANTITY_TYPE(1, -1, -2, 0, QPressure)

constexpr QPressure pascal(1.0);
constexpr QPressure bar = 100000 * pascal;
constexpr QPressure psi = pound * G / inch2;

inline namespace literals {
constexpr QPressure operator"" _Pa(long double x) {
  return QPressure(x);
}
constexpr QPressure operator"" _Pa(unsigned long long int x) {
  return QPressure(static_cast<double>(x));
}
constexpr QPressure operator"" _bar(long double x) {
  return static_cast<double>(x) * bar;
}
constexpr QPressure operator"" _bar(unsigned long long int x) {
  return static_cast<double>(x) * bar;
}
constexpr QPressure operator"" _psi(long double x) {
  return static_cast<double>(x) * psi;
}
constexpr QPressure operator"" _psi(unsigned long long int x) {
  return static_cast<double>(x) * psi;
}
} // namespace literals
} // namespace rev

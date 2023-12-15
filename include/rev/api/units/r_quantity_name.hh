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
#include "rev/api/units/q_angle.hh"
#include "rev/api/units/q_length.hh"
#include "rev/api/units/q_speed.hh"
#include <stdexcept>
#include <typeindex>
#include <unordered_map>

#pragma once

namespace rev {

/**
* Returns a short name for a unit.
* For example: `str(1_ft)` will return "ft", so will `1 * foot` or `0.3048_m`.
* Throws std::domain_error when `q` is a unit not defined in this function.
*
* @param q Your unit. Currently only QLength and QAngle are supported.
* @return The short string suffix for that unit.
*/
template <class QType> std::string getShortUnitName(QType q) {
  const std::unordered_map<std::type_index, std::unordered_map<double, const char *>> shortNameMap =
    {{typeid(meter),
      {
        {meter.get_value(), "m"},
        {decimeter.get_value(), "dm"},
        {centimeter.get_value(), "cm"},
        {millimeter.get_value(), "mm"},
        {kilometer.get_value(), "km"},
        {inch.get_value(), "in"},
        {foot.get_value(), "ft"},
        {yard.get_value(), "yd"},
        {mile.get_value(), "mi"},
        {tile.get_value(), "tile"},
      }},
     {typeid(degree), {{degree.get_value(), "deg"}, {radian.get_value(), "rad"}}}};

  try {
    return shortNameMap.at(typeid(q)).at(q.get_value());
  } catch (const std::out_of_range &e) {
    throw std::domain_error(
      "You have requested the shortname of an unknown unit somewhere (likely odometry strings). "
      "Shortname for provided unit is unspecified. You can override this function to add more "
      "names or manually specify the name instead.");
  }
}
} // namespace rev

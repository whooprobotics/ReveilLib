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

#include "rev/api/units/QArea.hpp"
#include "rev/api/units/QLength.hpp"
#include "rev/api/units/RQuantity.hpp"

namespace rev {
QUANTITY_TYPE(0, 3, 0, 0, QVolume)

constexpr QVolume kilometer3 = kilometer2 * kilometer;
constexpr QVolume meter3 = meter2 * meter;
constexpr QVolume decimeter3 = decimeter2 * decimeter;
constexpr QVolume centimeter3 = centimeter2 * centimeter;
constexpr QVolume millimeter3 = millimeter2 * millimeter;
constexpr QVolume inch3 = inch2 * inch;
constexpr QVolume foot3 = foot2 * foot;
constexpr QVolume mile3 = mile2 * mile;
constexpr QVolume litre = decimeter3;
} // namespace rev

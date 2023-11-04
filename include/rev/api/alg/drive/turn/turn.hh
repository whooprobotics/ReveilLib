#pragma once

#include "rev/api/units/all_units.hh"

namespace rev {  // The namespace is just to separate all of the reveillib code
                 // from
// everything else, so we can make sure names dont conflict
//  As a general rule, everything thats part of reveillib will be in this
//  namespace
class Turn {
 public:
  virtual void turn_to_target_absolute(double max_Power, QAngle angle) = 0;
};

enum class TurnState { INACTIVE, FULLPOWER, COAST, BRAKE };

}  // namespace rev

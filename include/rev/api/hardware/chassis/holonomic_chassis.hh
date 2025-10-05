#pragma once
#include "rev/api/hardware/chassis/chassis.hh"
#include "rev/api/alg/slipstream/power.hh"

namespace rev {
  /**
   * @brief Interface for holonomic chassis objects
   * 
   */

   class HolonomicChassis : public Chassis {
    public:
     virtual void drive_holonomic(SlipstreamPower power) = 0;
     virtual void drive_holonomic(double forward, double yaw, double strafe) = 0;
   };
}
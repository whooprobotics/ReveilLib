#pragma once
#include "rev/api/hardware/chassis/chassis.hh"

namespace rev {
  /**
   * @brief Interface for holonomic chassis objects
   * 
   */

   class HolonomicChassis : Chassis {
    public:
     virtual void drive_tank(double left, double right) = 0;

     virtual void drive_arcade(double forward, double yaw) = 0;

     virtual void drive_holonomic();

     virtual void set_brake_harsh() = 0;

     virtual void set_brake_coast() = 0;

     virtual void stop() = 0;
   };
}
#pragma once

#ifdef PLATFORM_BRAIN

#include "rev/api/v5/hardware/chassis/chassis.hh"
#include "rev/api/v5/alg/slipstream/power.hh"

namespace rev {
  /**
   * @brief Interface for holonomic chassis objects
   * 
   */

   class HolonomicChassis : public Chassis {
    public:
     /** 
      * @brief Drive the robot in a mecanum drive style
      * 
      * @param power Struct storing all motor powers for holonomic drives
      */
     virtual void drive_holonomic(SlipstreamPower power) = 0;

     /**
      * @brief Heuristic function for a holonomic drive from driver controls
      * 
      * @param forward The amount the robot is to move forward
      * 
      * @param yaw The amount the robot is to turn
      * 
      * @param strafe The amount the robot is to strafe
      */
     virtual void drive_holonomic(double forward, double yaw, double strafe) = 0;
   };
}

#endif
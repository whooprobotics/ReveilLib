#pragma once

#ifdef PLATFORM_BRAIN
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/hardware/chassis/chassis.hh"

namespace rev {

class ChassisSim : public Chassis, public Odometry {};

}  // namespace rev
#endif
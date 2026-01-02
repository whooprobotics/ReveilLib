#pragma once

#ifdef PLATFORM_BRAIN
#include "rev/api/v5/alg/odometry/odometry.hh"
#include "rev/api/v5/hardware/chassis/chassis.hh"

namespace rev {

class ChassisSim : public Chassis, public Odometry {};

}  // namespace rev
#endif
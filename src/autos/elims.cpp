#ifdef PLATFORM_BRAIN

#include "rev/rev.hh"
#include "autos/autos.h"
#include "robot_testing/rev2_config.hh"
#include "robot_testing/rev2_macros.hh"

using namespace rev;

void elims_right(){
    odom->set_position({-47_in, -16.75_in, 90_deg});

    driveTo(-16.5_in, -24.25_in, 0_deg);
    driveTo(0_in, -58.5_in, 180_deg);

    //Intake during next segment
    driveTo(0_in, -16.9_in, 180_deg);
    driveTo(0_in, -36.5_in, 180_deg);
    driveTo(-18.7_in, -16.9_in, 45_deg);
    driveTo(-8.9_in, -9.35_in, 45_deg);
}
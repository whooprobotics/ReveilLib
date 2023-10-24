
// You don't need a #pragma once here, because this isn't a header file and won't be included into anything
#include "rev/api/alg/drive/turn/campbell_turn.hh"
#include "rev/api/alg/odometry/odometry.hh"
namespace rev {
// The "CampbellTurn::" here just tells it that we are implementing a method that is a member of the CampbellTurn class
CampbellTurn::CampbellTurn(std::shared_ptr<Chassis> ichassis, std::shared_ptr<Odometry> iodometry, double ikP1, double ikP2, int icontroller_state) {
// Implement initialization stuff here, basically set up class member variables
kP1 = ikP1;
kP2 = ikP2;
controller_state = icontroller_state;
chassis = ichassis;
odometry = iodometry;
}
void CampbellTurn::turn_to_target_absolute(double max_Power, QAngle angle) {
// Implement the **NON-BLOCKING** turn to target absolute function
// This function should not wait for anything to happen, it should just set up variables and set the state or whatever and let the controller do its thing
}
void CampbellTurn::step() {
// Implement the actual controller logic
// This should basically behave as a "state transition" function
// It should not block execution, instead it should just make its checks, update the motor outputs if needed, and immediately return
// How to achieve that is up to you
}
// You will need implementations for every method
};
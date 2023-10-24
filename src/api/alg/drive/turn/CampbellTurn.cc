
// You don't need a #pragma once here, because this isn't a header file and won't be included into anything
#include "rev/api/alg/drive/turn/campbell_turn.hh"
#include "rev/api/alg/odometry/odometry.hh"
namespace rev {
// The "CampbellTurn::" here just tells it that we are implementing a method that is a member of the CampbellTurn class
CampbellTurn::CampbellTurn(std::shared_ptr<Chassis> ichassis, std::shared_ptr<Odometry> iodometry, double ikP1, double ikP2) {
// Implement initialization stuff here, basically set up class member variables
kP1 = ikP1;
kP2 = ikP2;
chassis = ichassis;
odometry = iodometry;

double angle_difference;
double target_relative_original;
double target_relative;

int left_direction = 1;
int right_direction = -1;
}
void CampbellTurn::turn_to_target_absolute(double max_Power, QAngle angle) {
// Implement the **NON-BLOCKING** turn to target absolute function
// This function should not wait for anything to happen, it should just set up variables and set the state or whatever and let the controller do its thing
controller_state = TurnState::FULLPOWER;
}
void CampbellTurn::step() {
    //OdometryState state = odometry->get_state();

    chassis->drive_tank(0,0);
    if (controller_state == TurnState::FULLPOWER){
        chassis->drive_tank(0,0);
    }
// Implement the actual controller logic
// This should basically behave as a "state transition" function
// It should not block execution, instead it should just make its checks, update the motor outputs if needed, and immediately return
// How to achieve that is up to you
}
// You will need implementations for every method
};
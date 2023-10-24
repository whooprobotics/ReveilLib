
// You don't need a #pragma once here, because this isn't a header file and won't be included into anything
#include "rev/api/alg/drive/turn/campbell_turn.hh"
#include "rev/api/alg/odometry/odometry.hh"
namespace rev {
// The "CampbellTurn::" here just tells it that we are implementing a method that is a member of the CampbellTurn class
CampbellTurn::CampbellTurn(std::shared_ptr<Chassis> ichassis, std::shared_ptr<Odometry> iodometry, double ikP1, double ikP2) {
// Implement initialization stuff here, basically set up class member variables
kP1 = ikP1; // was 0.2 in testing
kP2 = ikP2; // was 0.05 in testing
chassis = ichassis;
odometry = iodometry;

double angle_difference;
double target_relative_original;
double target_relative;


}
void CampbellTurn::turn_to_target_absolute(double imax_Power, QAngle iangle) {
// Implement the **NON-BLOCKING** turn to target absolute function
// This function should not wait for anything to happen, it should just set up variables and set the state or whatever and let the controller do its thing
double coast_turn_power = 0.175; // 0.175 from testing
int left_direction = 1;
int right_direction = -1;
double max_Power = imax_Power;
QAngle angle_goal = iangle;
controller_state = TurnState::FULLPOWER;

//** NEED TO CHECK ANGLE DIFFERENCE AND UPDATE MOTOR DIRECTION
}
void CampbellTurn::step() {
    //OdometryState state = odometry->get_state();

    // Full power turn
    if (controller_state == TurnState::FULLPOWER){
        //chassis->drive_tank(max_Power * left_direction * 1.0, max_Power * right_direction * 1.0);
    }
    // Low power turn
    else if (controller_state == TurnState::COAST)
    {
        //chassis->drive_tank(left_direction * coast_turn_power, right_direction * coast_turn_power);
    }
    // Activating hard brakes
    else if (controller_state == TurnState::BRAKE)
    {
        /* BRAKE */
    }
    // Disable motors/no power (normal coast mode)
    else if (controller_state == TurnState::INACTIVE)
    {
        chassis->drive_tank(0,0);
    }
    
    // Check Current angle/angular velocity and set controller_state
    
    
// Implement the actual controller logic
// This should basically behave as a "state transition" function
// It should not block execution, instead it should just make its checks, update the motor outputs if needed, and immediately return
// How to achieve that is up to you
}
// You will need implementations for every method
};
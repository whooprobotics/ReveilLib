
// You don't need a #pragma once here, because this isn't a header file and won't be included into anything
#include "rev/api/alg/drive/turn/campbell_turn.hh"
#include "rev/api/alg/odometry/odometry.hh"
#include "api.h"
namespace rev {
// The "CampbellTurn::" here just tells it that we are implementing a method that is a member of the CampbellTurn class
CampbellTurn::CampbellTurn(std::shared_ptr<Chassis> ichassis, std::shared_ptr<Odometry> iodometry, double ikP1, double ikP2) {
// Implement initialization stuff here, basically set up class member variables
kP1 = ikP1; // was 0.2 in testing
kP2 = ikP2; // was 0.05 in testing
chassis = ichassis;
odometry = iodometry;




}
void CampbellTurn::turn_to_target_absolute(double imax_Power, QAngle iangle) {
// Implement the **NON-BLOCKING** turn to target absolute function
// This function should not wait for anything to happen, it shoulmax_Powerd just set up variables and set the state or whatever and let the controller do its thing

max_Power = imax_Power;
QAngle angle_goal = iangle;
controller_state = TurnState::FULLPOWER;
angle_difference = angle_goal - odometry->get_state().pos.facing;
target_relative_original = angle_difference - 360 * std::floor((angle_difference.convert(degree) + 180) / 360)*degree;
target_relative = angle_difference - 360 * std::floor((angle_difference.convert(degree) + 180) / 360)*degree;

    if (angle_difference < 0*degree)
    { // if direction is negative, flip directions
        left_direction = -1;
        right_direction = 1;
    }
    else{
        left_direction = 1;
        right_direction = -1;
    }}
void CampbellTurn::step() {
    OdometryState state = odometry->get_state();

    // Full power turn
    if (controller_state == TurnState::FULLPOWER){
        chassis->drive_tank(left_direction * 1.0, max_Power * right_direction * 1.0);
    }
    // Low power turn
    else if (controller_state == TurnState::COAST)
    {
        chassis->drive_tank(left_direction * coast_turn_power, right_direction * coast_turn_power);
    }
    // Activating hard brakes
    else if (controller_state == TurnState::BRAKE)
    {
        chassis->set_brake_harsh();
        chassis->stop();
        if (brake_start_time == -1)
            brake_start_time = pros::millis();
        
       // pros::delay(250); //pros::delay bad >:(
        if (brake_start_time < pros::millis() - 250 ){ //250 ms until brake turns off
            chassis->set_brake_coast();
            controller_state = TurnState::INACTIVE; // set inactive and =-1 so it can be called again
            brake_start_time = -1;
        }
        
    }
    // Disable motors/no power (normal coast mode) // is bad -> can interfer with other code trying to manage chassis
    // else if (controller_state == TurnState::INACTIVE)
    // {
    //     chassis->drive_tank(0,0);
    // }
    
    // Check Current angle/angular velocity and set controller_state
    target_relative = angle_difference - 360 * std::floor((angle_difference.convert(degree) + 180) / 360)*degree;
    if  (fabs(target_relative.convert(degree)) > fabs(odometry->get_state().vel.angular.convert(degree/second) * kP1) && controller_state == TurnState::FULLPOWER){
        controller_state = TurnState::FULLPOWER;
    }
    else if  (fabs(target_relative.convert(degree)) > fabs(odometry->get_state().vel.angular.convert(degree/second) * kP2) && controller_state != TurnState::BRAKE){
        controller_state = TurnState::COAST;
    }
    else{
        controller_state = TurnState::BRAKE;
    }

    
// Implement the actual controller logic
// This should basically behave as a "state transition" function
// It should not block execution, instead it should just make its checks, update the motor outputs if needed, and immediately return
// How to achieve that is up to you
}
// You will need implementations for every method
};
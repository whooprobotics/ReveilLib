#pragma once

#include <memory>
#include "rev/api/alg/slipstream/power.hh"
#include "rev/api/hardware/chassis/holonomic_chassis.hh"
#include "rev/api/hardware/chassis/mecanum_chassis.hh"
#include "rev/api/hardware/motor/any_motor.hh"
#include "rev/rev.hh"

namespace rev {

    /**
     * @brief Implementation for a butterfly chassis extended from mecanum chassis
     * 
     */
    class ButterflyChassis : public MecanumChassis {
        public:

            /**
             * @brief Constructs a new butterfly chassis, motor groups are passed to mecanum constructor
             * 
             * @param ifront_left front left motor group
             * @param iback_left back left motor group
             * @param ifront_right front right motor group
             * @param iback_right back right motor group
             * 
             * @param left_piston left side piston for traction wheel actuation 
             * @param right_piston right side piston for traction wheel actuation 
             */
            ButterflyChassis(rev::AnyMotor& ifront_left, rev::AnyMotor& iback_left, 
                 rev::AnyMotor& ifront_right, rev::AnyMotor& iback_right, pros::ADIDigitalOut& left_piston, pros::ADIDigitalOut& right_piston);

            /**
             * @brief Drive method that switches between drive_holonomic and drive_arcade based on piston state
             * 
             * @param forward forward joystick input
             * @param yaw rotational joystick input
             * @param strafe lateral joystick input
             */
            void drive_auto(double forward, double yaw, double strafe);
            
            /**
             * @brief Toggles piston state (deployed <--> not deployed)
             */
            void toggle_pistons();

            /**
             * @brief sets piston state to value (true -> deployed) (false -> not deployed)
             * 
             * @param piston_val bool value that pistons are set to
             */
            void set_pistons(bool piston_val);

            /**
             * @brief return state of pistons (true -> deployed) (false -> not deployed)
             */
            bool get_pistons();

            private:
                std::shared_ptr<rev::AnyMotor> front_left;
                std::shared_ptr<rev::AnyMotor> back_left;
                std::shared_ptr<rev::AnyMotor> front_right;
                std::shared_ptr<rev::AnyMotor> back_right;
                std::shared_ptr<pros::ADIDigitalOut> left_piston;
                std::shared_ptr<pros::ADIDigitalOut> right_piston;
                bool pistons_down;
    };




}
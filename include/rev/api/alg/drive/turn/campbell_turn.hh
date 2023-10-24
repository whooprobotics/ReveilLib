#pragma once

#include "rev/api/alg/drive/turn/turn.hh"
#include "rev/api/hardware/chassis/chassis.hh" // For chassis model
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/async/async_runnable.hh"
#include <memory>

namespace rev {
class CampbellTurn : public Turn, public AsyncRunnable { // Rename this if you want
public:
 CampbellTurn(std::shared_ptr<Chassis> ichassis, std::shared_ptr<Odometry> iodometry, double ikP1, double iKp2, int icontroller_state); // Constructor function
 void turn_to_target_absolute(double max_Power, QAngle angle) override; // You need one of these for each virtual method in your interface
 void step() override; // This is also needed because it is an Async thing
private:
 std::shared_ptr<Chassis> chassis;
 std::shared_ptr<Odometry> odometry;
 // You can include other globals your controller relies on here. These will be referred to as class members
 // A good example would be to put your kP1 and kP2 values here too
 double kP1;
 double kP2;
 int controller_state;
 // If you do that, you will also need to add them to your constructor signature so you can set them in the constructor
 // It would also be a good idea to have an int or something to store the current state of the controller here. You'll see why in a minute

 // Also having a shared pointer to the odometry controller would be a good idea so you know where the robot is pointing
};
}
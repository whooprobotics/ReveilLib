#pragma once

class Chassis {
    virtual void drive(double left, double right) = 0;
    virtual void driveVector(double forward, double yaw) = 0;
};
# Reckless Controller Usage Guide

## Getting Started

Before you can use the reckless controller, you need to get a few things out of the way

### Includes

In order to use Reckless, you need ReveilLib. You can do this by including rev.hh

```cpp
#include "rev/rev.hh"
```

### Chassis

The Reckless controller is designed for use with a skid-steer chassis, so that is what this guide will focus on. The skid-steer chassis relies on PROS motor groups. Below is an example configuration with 2 motor groups, one for the left side of the chassis and one for the right side, as used with Slingo, the WHOOP Spin Up 15 inch robot.

```cpp
pros::Motor_Group leftd({15, 18, -19, -20});
pros::Motor_Group rightd({4, 6, -7, -9});
```

With the motor groups defined, you can build your chassis model

```cpp
std::shared_ptr<rev::SkidSteerChassis> chassis =
    std::make_shared<rev::SkidSteerChassis>(leftd, rightd);
```

If you don't want to use a skid steer chassis, you can also define your own compatible chassis model by implementing the `rev::Chassis` interface. As of 1.0.0, ReveilLib does not support any models other than skid steer.

### Odometry

In addition to a chassis, an odometry system is also required. Like with chassis, a compatible custom odometry solution can be created by extending `rev::Odometry`, however by default ReveilLib 1.0.0 only comes with a single odometry system type.

The TwoEncoderGyroOdometry system uses 3 sensors; an inertial and 2 rotation sensors. These can be set up as below:

```cpp
pros::Rotation fwd(5);        // Rotation sensor connected to the wheel measuring forward translation
pros::Rotation rgt(16, true); // Rotation sensor connected to the wheel measuring rightward translation. 
                              // This one had to be reversed because of the way the sensor was mounted.
                              // NOTE: Unlike motors, the PROS kernel requires you to use (port, true) syntax to initialize a reversed rotation sensor. If instead you try using (-port), it will not work.
pros::Imu imu(14); // Inertial sensor
```

Using this, you can then build an odometry system!

```cpp
std::shared_ptr<rev::TwoRotationInertialOdometry> odom =
  std::make_shared<rev::TwoRotationInertialOdometry>(
    fwd,      // The forward sensor
    rgt,      // The rightward sensor 
    imu,      // Inertial sensor
    2.09_in,  // Diameter of forward wheel
    2.75_in,  // Diameter of sideways wheel
    4.75_in,  // How far to the right of the center of the robot the forward wheel is
    0.5_in    // How far to the rear of the robot the lateral wheel is from the center
  );
```

Additionally, you will need to initalize the odometry, as it has an associated thread. This can be done at the top of your autonomous function as follows
```cpp
rev::AsyncRunner odom_runner(odom);
```

The inertial sensor automatically calibrates every time a Vex program starts, so it might be wise to wait a few seconds before creating the runner.

### Reckless controller

Finally you can build the Reckless controller itsself.

```cpp
std::shared_ptr<rev::Reckless> reckless =
      std::make_shared<Reckless>(chassis, odom);
```

and to make it run, just add this to the top of your autonomous function

```cpp
AsyncRunner reckless_runner(reckless);
```

## Using the Reckless controller

The Reckless controller accepts a series of path segments for it to follow. An example path is shown below

```cpp
reckless->go(
  RecklessPath()
    .with_segment(RecklessPathSegment(
        std::make_shared<CascadingMotion>(0.7, kP, kB,
                                          60_in / second, 0.07),
        std::make_shared<PilonsCorrection>(2, 0.5_in),
        std::make_shared<SimpleStop>(0_s, 0.3_s, 0.2),
        {-2_ft, 0_ft, 0_deg}, 0_in))
    .with_segment(RecklessPathSegment(
        std::make_shared<CascadingMotion>(0.7, kP, kB,
                                          60_in / second, 0.07),
        std::make_shared<PilonsCorrection>(2, 0.5_in),
        std::make_shared<SimpleStop>(.1_s, 0.2_s, 0.4),
        {-4_ft, -1_ft, 45_deg}, 0_in))
    .with_segment(RecklessPathSegment(
        std::make_shared<CascadingMotion>(0.7, kP, kB,
                                          60_in / second, 0.07),
        std::make_shared<PilonsCorrection>(2, 0.5_in),
        std::make_shared<SimpleStop>(0.075_s, 0.2_s, 0.4),
        {0_ft, 0_ft, 0_deg}, 0_in)
));
```

While the motion, correction, and stopping algorithms can be overridden for each segment of the path, ReveilLib comes with 3 motion algorithms, 2 correction algorithms, and 1 stop algorithm built in

### Motion

ReveilLib's 3 motion algorithms are `Constant`, `Proportional`, and `Cascading`.

`ConstantMotion` is the simplest of the 3, and just outputs a constant power. It can be created with

```cpp
std::make_shared<ConstantMotion>(power)
```

where you can replace `power` with whatever power you wish to use, from 0 to 1


The next motion algorithm is `ProportionalMotion`. This one is slightly more complicated, and has 2 parameters; a proportional gain and a max power. It can be made as follows:

```cpp
std::make_shared<ProportionalMotion>(max_power, k_p)
```

where `max_power` is the max power and `k_p` is the proportional gain.

Finally, `CascadingMotion` is the most complicated, with 5 parameters. This one attempts to move at a consistent speed, unlike ProportionalMotion, which only cares about voltage output.

```cpp
std::make_shared<CascadingMotion>(max_power, k_p, k_b, max_speed, k_v)
```

`max_power` is the maximum power the controller will output, from 0 to 1.

`k_p` is a constant of proportionality aiming to increase the robot's speed if it is below its target speed or slow it down if it's going too fast.

`k_b` is a feed-forward proportional gain aiming to make the robot move at a specific speed.

`max_v` is a speed limit for the robot to target.

`k_v` is a constant that determines how the robot will choose its target speed. `0.07` seems to work well.

### Correction

2 correction algorithms are provided

`NoCorrection` is self-explainatory. It doesn't do anything.

`PilonsCorrection` is an algorithm based on 5225A's In The Zone code. It can be made as following:

```cpp
std::make_shared<PilonsCorrection>(k_correction, max_error);
```

`k_correction` is the proportional gain for correction. `2` seems to work well.

`max_error` is a length expressing how far off the robot can be before it starts attempting to correct. `0.5_in` works well in testing.

### Stopping

We also need a stop controller. A simple one is provided.

```cpp
std::make_shared<SimpleStop>(k_h, k_c, pow_coast)
```

`k_h` is a time such that if the robot continues at its current velocity and it will reach the target before that much more time passes, it will apply its harsh brakes.

`k_c` is similar, but it will coast instead.

`pow_coast` is the amount of power thats fed to the motors while coasting. This should be just enough to allow the robot to overcome friction and rolling resistance.

### Target point

The target point is the fifth parameter of a segment. Only the `x` and `y` matter however, as the robot really doesn't care where its facing (this can be corrected later using `CampbellTurn` though).

### Early Drop

The early drop parameter allows you to make the robot begin execution of the next segment before it reaches the target point of the current one. This is particularly useful for making motions smoother. It just accepts a `QLength`.

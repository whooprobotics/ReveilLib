# The Reckless Controller

The Reckless Controller is the flagship feature of ReveilLib. It is designed to be an all-in-one robot control system for autonomous, capable of directing the robot's various subsystems to perform various actions. This document will attempt to explain how it works.

# Reckless Segments

The Reckless Controller works by executing Reckless Segments. What is a Reckless segment? Well, it is just... something for the Reckless controller to execute.

A Reckless segment implements 3 basic functions:

```cpp
void init(OdometryState);

SegmentStatus step(OdometryState);

void clean_up(void);
```

These 3 methods should all be "non-blocking", that is, they should take little to no time, and if they are going to kick off an action that will take a long time, such as raising an arm, they should signal to some other task to perform the action and return immediately. The reasoning behind this design will become apparent later.

The `init` method is executed once when the segment begins. It is mostly used to set up initial variables and perform calculations that only need to be performed once.

The `step` method is called on every loop of the Reckless controller while the segment is being executed, or roughly every 10 milliseconds by default. It is used mostly to control the chassis. This is where the bulk of the logic for a controller will typically be.

The `cleanup` method is used to do some sort of task at the end of a segment.

You might notice that unlike the other 2 methods, the `step` method has a return value. This is because the `step` method needs to be able to communicate with the Reckless controller, in order to move the chassis motors and to handoff control to the next segment when ready.

The return type of `step` is `SegmentStatus`. This is a special type that is able to communicate data using a concept borrowed from Haskell and Rust called an Algebraic Data Type. Below is an abbreviated version of the source code for the segment status. The first thing you will notice is a type called `SegmentStatusType`. This is encoded into the segment status, and is used for signaling the action which should be taken by the Reckless Controller.

```cpp
enum class SegmentStatusType { DRIVE, BRAKE, NEXT, DUMMY };
struct SegmentStatus {
  SegmentStatusType status;
  double power_left{0.};
  double power_right{0.};

  static SegmentStatus drive(double ipower_left, double ipower_right) {
    SegmentStatus status;
    status.status = SegmentStatusType::DRIVE;
    status.power_left = ipower_left;
    status.power_right = ipower_right;

    return status;
  }

  static SegmentStatus brake() {
    SegmentStatus status;
    status.status = SegmentStatusType::BRAKE;
    return status;
  }

  static SegmentStatus next() {
    SegmentStatus status;
    status.status = SegmentStatusType::NEXT;
    return status;
  }

  static SegmentStatus dummy() {
    SegmentStatus status;
    status.status = SegmentStatusType::DUMMY;
    return status;
  }
};
```

The `SegmentStatus` class also has some internal variables which are used to pass the drive values in the event that a `DRIVE` state is used.

It also comes with some static member functions which construct the algebraic data type as needed, those being `drive, brake, next, dummy`.

Now lets examine what the Reckless controller does with these.


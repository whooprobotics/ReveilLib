#pragma once

#include "mecanum_util.hh"

extern Constants constants;

struct pid_params {
    double p;
    double i;
    double d;
    double starti;
};

struct settle_params {
    double settle_error;
    double settle_time;
    double large_settle_error;
    double large_settle_time;
};

// Defaulting parameters to the constants struct
struct mecanum_to_pose_params {
    pid_params drive_k = {
        .p = constants.drive_kp,
        .i = constants.drive_ki,
        .d = constants.drive_kd,
        .starti = constants.drive_starti
    };
    pid_params turn_k = {
        .p = constants.heading_kp,
        .i = constants.heading_ki,
        .d = constants.heading_kd,
        .starti = constants.heading_starti
    };

    settle_params turn_settle = {
        .settle_error = constants.drive_settle_error,
        .settle_time = constants.drive_settle_time,
        .large_settle_error = constants.drive_large_settle_error,
        .large_settle_time = constants.drive_large_settle_time
    };

    settle_params drive_settle = {
        .settle_error = constants.turn_settle_error,
        .settle_time = constants.turn_settle_time,
        .large_settle_error = constants.turn_large_settle_error,
        .large_settle_time = constants.turn_large_settle_time
    };

    double exit_error = constants.drive_exit_error;
    double min_speed = constants.drive_min_speed;
    double max_speed = constants.drive_max_speed;
    double turn_max_speed = constants.turn_max_speed;
    double timeout = constants.drive_timeout;

};

void mecanum_to_pose(double x, double y, double angle, mecanum_to_pose_params p = mecanum_to_pose_params{});

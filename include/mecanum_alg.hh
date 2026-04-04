#pragma once

#include "mecanum_util.hh"
#include <ostream>

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
        .p = constants.turn_kp,
        .i = constants.turn_ki,
        .d = constants.turn_kd,
        .starti = constants.turn_starti
    };

    settle_params drive_settle = {
        .settle_error = constants.drive_settle_error,
        .settle_time = constants.drive_settle_time,
        .large_settle_error = constants.drive_large_settle_error,
        .large_settle_time = constants.drive_large_settle_time
    };

    settle_params turn_settle = {
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

struct mecanum_turn_to_angle_params {
    pid_params turn_k = {
        .p = constants.turn_kp,
        .i = constants.turn_ki,
        .d = constants.turn_kd,
        .starti = constants.turn_starti
    };

    settle_params turn_settle = {
        .settle_error = constants.turn_settle_error,
        .settle_time = constants.turn_settle_time,
        .large_settle_error = constants.turn_large_settle_error,
        .large_settle_time = constants.turn_large_settle_time
    };

    TurnDirection turn_direction = TurnDirection::NONE;
    double exit_error = constants.turn_exit_error;
    double min_speed = constants.turn_min_speed;
    double max_speed = constants.turn_max_speed;
    double timeout = constants.turn_timeout;
};

void mecanum_turn_to_angle(double angle, mecanum_turn_to_angle_params p = mecanum_turn_to_angle_params{});

inline std::ostream& operator<<(std::ostream& os, const pid_params& p) {
    return os << "pid_params{ p=" << p.p << ", i=" << p.i << ", d=" << p.d << ", starti=" << p.starti << " }";
}

inline std::ostream& operator<<(std::ostream& os, const settle_params& s) {
    return os << "settle_params{ settle_error=" << s.settle_error
              << ", settle_time=" << s.settle_time
              << ", large_settle_error=" << s.large_settle_error
              << ", large_settle_time=" << s.large_settle_time << " }";
}

inline std::ostream& operator<<(std::ostream& os, const mecanum_to_pose_params& p) {
    return os << "mecanum_to_pose_params{\n"
              << "  drive_k=" << p.drive_k << "\n"
              << "  turn_k=" << p.turn_k << "\n"
              << "  drive_settle=" << p.drive_settle << "\n"
              << "  turn_settle=" << p.turn_settle << "\n"
              << "  exit_error=" << p.exit_error
              << ", min_speed=" << p.min_speed
              << ", max_speed=" << p.max_speed
              << ", turn_max_speed=" << p.turn_max_speed
              << ", timeout=" << p.timeout << "\n"
              << "}";
}



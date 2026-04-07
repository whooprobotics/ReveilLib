#pragma once

#include "mecanum_util.hh"
#include "mecanum_units.hh"
#include "mecanum_PID.hh"
#include <ostream>

extern Constants constants;

struct mecanum_to_pose_params {
    pidParams<QLength> drive_k = {
        .p = constants.drive_kp,
        .i = constants.drive_ki,
        .d = constants.drive_kd,
        .starti = constants.drive_starti
    };
    pidParams<QAngle> turn_k = {
        .p = constants.turn_kp,
        .i = constants.turn_ki,
        .d = constants.turn_kd,
        .starti = constants.turn_starti
    };

    settleParams<QLength> drive_settle = {
        .settle_error = constants.drive_settle_error,
        .settle_time = constants.drive_settle_time,
        .large_settle_error = constants.drive_large_settle_error,
        .large_settle_time = constants.drive_large_settle_time
    };

    settleParams<QAngle> turn_settle = {
        .settle_error = constants.turn_settle_error,
        .settle_time = constants.turn_settle_time,
        .large_settle_error = constants.turn_large_settle_error,
        .large_settle_time = constants.turn_large_settle_time
    };

    QLength exit_error = constants.drive_exit_error;
    double min_speed = constants.drive_min_speed;
    double max_speed = constants.drive_max_speed;
    double turn_max_speed = constants.turn_max_speed;
    QTime timeout = constants.drive_timeout;
};

void mecanum_to_pose(QLength x, QLength y, QAngle angle, mecanum_to_pose_params p = mecanum_to_pose_params{});

struct mecanum_turn_to_angle_params {
    pidParams<QAngle> turn_k = {
        .p = constants.turn_kp,
        .i = constants.turn_ki,
        .d = constants.turn_kd,
        .starti = constants.turn_starti
    };

    settleParams<QAngle> turn_settle = {
        .settle_error = constants.turn_settle_error,
        .settle_time = constants.turn_settle_time,
        .large_settle_error = constants.turn_large_settle_error,
        .large_settle_time = constants.turn_large_settle_time
    };

    TurnDirection turn_direction = TurnDirection::NONE;
    QAngle exit_error = constants.turn_exit_error;
    double min_speed = constants.turn_min_speed;
    double max_speed = constants.turn_max_speed;
    QTime timeout = constants.turn_timeout;
};

void mecanum_turn_to_angle(QAngle angle, mecanum_turn_to_angle_params p = mecanum_turn_to_angle_params{});

template<typename T>
inline std::ostream& operator<<(std::ostream& os, const pidParams<T>& p) {
    return os << "pidParams{ p=" << p.p << ", i=" << p.i << ", d=" << p.d << ", starti=" << p.starti << " }";
}

template<typename T>
inline std::ostream& operator<<(std::ostream& os, const settleParams<T>& s) {
    return os << "settleParams{ settle_error=" << s.settle_error.internal()
              << ", settle_time=" << s.settle_time.internal()
              << ", large_settle_error=" << s.large_settle_error.internal()
              << ", large_settle_time=" << s.large_settle_time.internal() << " }";
}

inline std::ostream& operator<<(std::ostream& os, const mecanum_to_pose_params& p) {
    return os << "mecanum_to_pose_params{\n"
              << "  drive_k=" << p.drive_k << "\n"
              << "  turn_k=" << p.turn_k << "\n"
              << "  drive_settle=" << p.drive_settle << "\n"
              << "  turn_settle=" << p.turn_settle << "\n"
              << "  exit_error=" << p.exit_error.internal()
              << ", min_speed=" << p.min_speed
              << ", max_speed=" << p.max_speed
              << ", turn_max_speed=" << p.turn_max_speed
              << ", timeout=" << p.timeout.internal() << "\n"
              << "}";
}

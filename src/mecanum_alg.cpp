#include "rev/rev.hh"
#include "mecanum_util.hh"
#include "mecanum_alg.hh"
#include "robot-config.hh"
#include <algorithm>

void mecanum_turn_to_angle(double angle, mecanum_turn_to_angle_params p) {
    PID turnPID(
        p.turn_k.p, p.turn_k.i, p.turn_k.d, p.turn_k.starti,
        p.turn_settle.settle_error, p.turn_settle.settle_time, p.turn_settle.large_settle_error, p.turn_settle.large_settle_time,
        p.exit_error, p.timeout
    );

    std::cout << p.turn_k << std::endl;

    double prev_raw_error = reduce_negative_180_to_180(angle - imu.get_heading());
    double prev_error = angle_error(angle - imu.get_heading(), p.turn_direction);
    bool crossed = false;

    while (!turnPID.is_settled()) {
        double raw_error = reduce_negative_180_to_180(angle - imu.get_heading());

        if (sign(raw_error) != sign(prev_raw_error)) {
            crossed = true;
        }
        prev_raw_error = raw_error;

        double error;
        if (crossed) {
            error = raw_error;
        } else {
            error = angle_error(angle - imu.get_heading(), p.turn_direction);
        }

        if (p.min_speed != 0 && crossed && sign(error) != sign(prev_error)) { break; }
        prev_error = error;

        double output = turnPID.compute(error);
        output = clamp(output, -p.max_speed, p.max_speed);
        output = clamp_min_voltage(output, p.min_speed) / 12;

        rev::SlipstreamPower power = {
            .front_left_forward  = output,
            .front_right_forward = -output,
            .rear_left_forward   = output,
            .rear_right_forward  = -output
        };

        chassis.drive_holonomic(power);
        pros::delay(10);
    }
}

void mecanum_to_pose(double x, double y, double angle, mecanum_to_pose_params p) {
    PID drivePID(
        p.drive_k.p, p.drive_k.i, p.drive_k.d, p.drive_k.starti, // PID constants
        p.drive_settle.settle_error, p.drive_settle.settle_time, p.drive_settle.large_settle_error, p.drive_settle.large_settle_time, // Settling parameters
        0, p.timeout // Exit parameters
    );

    PID turnPID(
        p.turn_k.p, p.turn_k.i, p.turn_k.d, p.turn_k.starti, // PID constants
        p.turn_settle.settle_error, p.turn_settle.settle_time, p.turn_settle.large_settle_error, p.turn_settle.large_settle_time, // Settling parameters
        0, p.timeout // Exit parameters
    );
    std::cout << p << std::endl;

    bool prev_line_settled = false;

    while (!(drivePID.is_settled() && turnPID.is_settled())) {

        float desired_heading = to_deg(atan2(x - odom45.get_x(), y - odom45.get_y()));
        
        bool line_settled = is_line_settled(x, y, desired_heading, odom45.get_x(), odom45.get_y(), p.exit_error);
        if (!(line_settled == prev_line_settled) && p.min_speed > 0) { break; }
        prev_line_settled = line_settled;

        float drive_error = hypot(x - odom45.get_x(), y - odom45.get_y());
        float turn_error = reduce_negative_180_to_180(angle - imu.get_heading());
        
        float drive_output = drivePID.compute(drive_error);
        float turn_output = turnPID.compute(turn_error);

        drive_output = clamp(drive_output, -p.max_speed, p.max_speed);
        turn_output = clamp(turn_output, -p.turn_max_speed, p.turn_max_speed);

        drive_output = clamp_min_voltage(drive_output, p.min_speed);
        turn_output = clamp_min_voltage(turn_output, p.min_speed);
        
        float heading_error = atan2(y - odom45.get_y(), x - odom45.get_x());
    
        float left_front_output  = (drive_output * cos(to_rad(imu.get_heading()) + heading_error - M_PI / 4) + turn_output) / 12;
        float left_back_output   = (drive_output * cos(-to_rad(imu.get_heading()) - heading_error + 3 * M_PI / 4) + turn_output) / 12;
        float right_back_output  = (drive_output * cos(to_rad(imu.get_heading()) + heading_error - M_PI / 4) - turn_output) / 12;
        float right_front_output = (drive_output * cos(-to_rad(imu.get_heading()) - heading_error + 3 * M_PI / 4) - turn_output) / 12;


        rev::SlipstreamPower power = {
          .front_left_forward = left_front_output,
          .front_right_forward = right_front_output,
          .rear_left_forward = left_back_output,
          .rear_right_forward = right_back_output,
        };

        chassis.drive_holonomic(power);

        pros::delay(10);
    }
}


#include "rev/rev.hh"
#include "mecanum_util.hh"
#include "mecanum_alg.hh"
#include "robot-config.hh"
#include <algorithm>

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
    
        float left_front_output  = (drive_output * cos(to_rad(imu.get_heading()) + heading_error - M_PI / 4) + turn_output);
        float left_back_output   = (drive_output * cos(-to_rad(imu.get_heading()) - heading_error + 3 * M_PI / 4) + turn_output);
        float right_back_output  = (drive_output * cos(to_rad(imu.get_heading()) + heading_error - M_PI / 4) - turn_output);
        float right_front_output = (drive_output * cos(-to_rad(imu.get_heading()) - heading_error + 3 * M_PI / 4) - turn_output);


        chassis.drive_holonomic(rev::SlipstreamPower{left_front_output, right_front_output, left_back_output, right_back_output, 0, 0});
    }
}

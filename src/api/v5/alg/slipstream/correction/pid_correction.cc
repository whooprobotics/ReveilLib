#ifdef PLATFORM_BRAIN

#include <cmath>
#include <algorithm>
#include "rev/rev.hh"
#include "rev/api/v5/alg/slipstream/correction/pid_correction.hh"
#include "rev/api/common/units/q_angle.hh"
#include "rev/api/common/units/q_length.hh"


namespace rev {

    PointVector rotateVectorByAngle(PointVector old_vector, QAngle angle){
        return PointVector{cos(angle) * old_vector.x - sin(angle) * old_vector.y, sin(angle)*old_vector.x + cos(angle)*old_vector.y};
    }

    PIDCorrection::PIDCorrection(double kp, double ki, double kd, QLength imax_forward_error,
                       QLength imax_lateral_error, QAngle imax_heading_error) 
                       : kp{kp}, ki{ki}, kd{kd}, max_forward_error{imax_forward_error}, 
                       max_lateral_error{imax_lateral_error}, max_heading_error{imax_heading_error}, time{0.0}, 
                       integral_sum_forward{0.0}, integral_sum_lateral{0.0}, integral_sum_heading{0.0}, last_forward_error{0.0}, 
                       last_lateral_error{0.0}, last_heading_error{0.0} {}
    
    SlipstreamPower PIDCorrection::apply_correction(OdometryState current_state, Position target_state, Position start_state, QLength drop_early,
    SlipstreamPower powers) {
        //Calculates drive vector with target pose - current pose and rotates it by heading
        //rotated_vector.x is forward error, rotated_vector.y is lateral error and heading_diff is heading error
        Position current_pos = current_state.pos;
        PointVector drive_vector = {target_state.x - current_pos.x, target_state.y - current_pos.y};
        PointVector rotated_vector = rotateVectorByAngle(drive_vector, current_pos.theta);
        QAngle heading_diff = target_state.theta - current_pos.theta;

        //Gets delta_time for integral and derivative terms
        double current_time = pros::millis();
        double delta_time = current_time - time;
        time = current_time;

        //Multiplies error vectors by kp to get p_correction
        double forward_p_correction = kp * rotated_vector.x.convert(inch);
        double lateral_p_correction = kp * rotated_vector.y.convert(inch);
        double heading_p_correction = kp * heading_diff.convert(radian);

        //Gets integral of error with riemann sum and multiplies it by ki to get i_correction
        integral_sum_forward += delta_time * rotated_vector.x.convert(inch);
        integral_sum_lateral += delta_time * rotated_vector.y.convert(inch);
        integral_sum_heading += delta_time * heading_diff.convert(radian);
        double forward_i_correction = ki * integral_sum_forward;
        double lateral_i_correction = ki * integral_sum_lateral;
        double heading_i_correction = ki * integral_sum_heading;

        //Gets derivative of error with current - last / delta_time and multiplies it by kd to get d_correction
        //Also reassigns last_error
        double forward_d_correction = kd * (rotated_vector.x.convert(inch) - last_forward_error) / delta_time;
        double lateral_d_correction = kd * (rotated_vector.y.convert(inch) - last_lateral_error) / delta_time;
        double heading_d_correction = kd * (heading_diff.convert(radian) - last_heading_error) / delta_time;
        last_forward_error = rotated_vector.x.convert(inch);
        last_lateral_error = rotated_vector.y.convert(inch);
        last_heading_error = heading_diff.convert(radian);

        //Sums correction
        double forward_correction = forward_p_correction + forward_i_correction + forward_d_correction;
        double lateral_correction = lateral_p_correction + lateral_i_correction + lateral_d_correction;
        double heading_correction = heading_p_correction + heading_i_correction + heading_d_correction;

        //Assigns corrections to wheel powers based on mecanum equation
        double fl_power = forward_correction + lateral_correction + heading_correction;
        double fr_power = forward_correction - lateral_correction - heading_correction;
        double bl_power = forward_correction - lateral_correction + heading_correction;
        double br_power = forward_correction + lateral_correction - heading_correction;
        
        // Normalizes wheel powers
        double max_power = std::max({std::abs(fl_power), std::abs(fr_power), std::abs(bl_power), std::abs(br_power)});
        if(max_power > 1.0){
            fl_power /= max_power;
            fr_power /= max_power;
            bl_power /= max_power;
            br_power /= max_power;
        }

        //assigns wheel powers to SlipstreamPower object
        SlipstreamPower corrected_powers;
        corrected_powers.front_left_forward = fl_power;
        corrected_powers.front_right_forward = fr_power;
        corrected_powers.rear_left_forward = bl_power;
        corrected_powers.rear_right_forward = br_power;

        return corrected_powers;
    }
    

}

#endif
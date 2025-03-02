#include "rev/api/alg/reckless/look_at.hh"
#include "api.h"
#include "rev/api/alg/reckless/segment.hh"
#include "rev/api/alg/reckless/turn_segment.hh"

// Wrapper of RecklessTurnSegment
// Takes a position value instead of angle_goal

namespace rev {

    RecklessLookAt::RecklessLookAt(
        double imax_power, 
        double icoast_power, 
        Position itarget_position, 
        QAngle idrop_angle, 
        double iharsh_coeff, 
        double icoast_coeff, 
        QTime ibrake_time) 
        : max_power(imax_power),
        coast_power(icoast_power),
        target_position(itarget_position),
        drop_angle(idrop_angle),
        harsh_coeff(iharsh_coeff),
        coast_coeff(icoast_coeff),
        brake_time(ibrake_time),
        turn_segment(imax_power, icoast_power, 0 * degree, iharsh_coeff, icoast_coeff, ibrake_time)
        {}

    void RecklessLookAt::init(OdometryState initial_state){
        start_position = initial_state.pos;

        // angle between start point and target
        double computed_angle = atan2(
            target_position.y - start_position.y
            ,
            target_position.x - start_position.x
        ).convert(degree);// * 180 / M_PI;

        // normalize angle between 2 points
        computed_angle = computed_angle - 360.0 * std::floor((computed_angle + 180.0) / 360.0);

        // the 2 drop off angles
        double angle1 = computed_angle - drop_angle.convert(degree);
        double angle2 = computed_angle + drop_angle.convert(degree);
        // normalize drop angles
        angle1 = angle1 - 360.0 * std::floor((angle1 + 180.0) / 360.0);
        angle2 = angle2 - 360.0 * std::floor((angle2 + 180.0) / 360.0);

        // select closer angle
        angle_goal = (
            fabs(angle1 - start_position.theta.convert(degree))
         < fabs(angle2 - start_position.theta.convert(degree))
        ) 
         ? angle1 * degree : angle2 * degree;

         // call RecklessTurnSegment with better angle
        turn_segment = RecklessTurnSegment(
            max_power, 
            coast_power, 
            angle_goal, 
            harsh_coeff, 
            coast_coeff, 
            brake_time
        );
        turn_segment.init(initial_state);
    }

    SegmentStatus RecklessLookAt::step(OdometryState current_state){
        return turn_segment.step(current_state);
    }

    void RecklessLookAt::clean_up() {
        turn_segment.clean_up();
    }
}
#pragma once
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/alg/reckless/segment.hh"
#include "rev/api/units/q_time.hh"
#include "rev/api/units/q_angle.hh"

namespace rev {

/**
 * @brief Path segment for turning for use with Reckless controller,
 * implementing Walker Campbell's turn algorithm
 * 
 */
class RecklessTurnSegment : public RecklessSegment {

  public:
    RecklessTurnSegment(double imax_power, double icoast_power,
                        QAngle iangle, double iharsh_coeff, double icoast_coeff, QTime ibrake_time);

    enum class TurnState { FULLPOWER, COAST, BRAKE };
    
    void init(OdometryState initial_state);

    SegmentStatus step(OdometryState current_state);

    void clean_up();

  private:
    double max_power;
    double harsh_coeff;
    double coast_coeff;
    double coast_power;
    uint32_t brake_time;
    QAngle angle_goal = 0_deg;
    QAngle start_angle;
    QAngle angle_difference;
    QAngle target_relative_original;
    QAngle target_relative;
    int left_direction = 0;
    int right_direction = 0;
    int brake_start_time = -1;
    TurnState controller_state{TurnState::FULLPOWER};
};

} // namespace rev

#pragma once
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/alg/drive/turn/turn.hh"
#include "rev/api/alg/reckless/segment.hh"

namespace rev {

/**
 * @brief Path segment for turning for use with Reckless controller,
 * implementing Walker Campbell's turn algorithm
 * 
 */
class RecklessTurnSegment : public RecklessSegment {

  public:
    RecklessTurnSegment(double imax_power, double icoast_power,
                        QAngle iangle, double iharsh_coeff, double icoast_coeff);
    
    void init(OdometryState initial_state);

    SegmentStatus step(OdometryState current_state);

    void clean_up();

  private:
    double max_power;
    double harsh_coeff;
    double coast_coeff;
    double coast_power;
    QAngle angle = 0_deg;
    QAngle angle_difference;
    QAngle target_relative_original;
    QAngle target_relative;
    int left_direction = 0;
    int right_direction = 0;
    int brake_start_time = -1;
    TurnState controller_state{TurnState::INACTIVE};
};

} // namespace rev

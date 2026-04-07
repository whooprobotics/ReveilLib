#pragma once
#include <memory>
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/alg/slipstream/segment.hh"
#include "rev/api/units/q_angle.hh"
#include "rev/api/units/q_length.hh"
#include "rev/api/units/q_time.hh"

namespace rev {

class LookAt : public SlipstreamSegment {
    public:
        LookAt(
            double imax_power, 
            double icoast_power, 
            Position itarget_position, 
            QAngle idrop_angle, 
            double iharsh_coeff, 
            double icoast_coeff, 
            QTime ibrake_time);

        LookAt(
            double imax_power, 
            double icoast_power, 
            Position itarget_position, 
            QAngle idrop_angle, 
            double iharsh_coeff, 
            double icoast_coeff, 
            QTime ibrake_time,
            QTime itimeout);

        LookAt(
            double imax_power, 
            double icoast_power, 
            Position itarget_position, 
            QAngle iangle_offset,
            QAngle idrop_angle, 
            double iharsh_coeff, 
            double icoast_coeff, 
            QTime ibrake_time);

        LookAt(
            double imax_power, 
            double icoast_power, 
            Position itarget_position, 
            QAngle iangle_offset,
            QAngle idrop_angle, 
            double iharsh_coeff, 
            double icoast_coeff, 
            QTime ibrake_time,
            QTime itimeout);

        void init(OdometryState initial_state) override;
        SlipstreamSegmentStatus step(OdometryState current_state) override;
        void clean_up() override;

        std::shared_ptr<SlipstreamSegment> operator&() {
            return std::make_shared<LookAt>(*this);
        }

        static std::shared_ptr<LookAt> create(
            double imax_power, 
            double icoast_power, 
            Position itarget_position, 
            QAngle idrop_angle, 
            double iharsh_coeff, 
            double icoast_coeff, 
            QTime ibrake_time) {
                return std::make_shared<LookAt>(
                    imax_power, icoast_power, itarget_position,
                    idrop_angle, iharsh_coeff, icoast_coeff, ibrake_time
                );
            }

        static std::shared_ptr<LookAt> create(
            double imax_power, 
            double icoast_power, 
            Position itarget_position, 
            QAngle idrop_angle, 
            double iharsh_coeff, 
            double icoast_coeff, 
            QTime ibrake_time,
            QTime itimeout) {
                return std::make_shared<LookAt>(
                    imax_power, icoast_power, itarget_position,
                    idrop_angle, iharsh_coeff, icoast_coeff, ibrake_time, itimeout
                );
            }

        static std::shared_ptr<LookAt> create(
            double imax_power, 
            double icoast_power, 
            Position itarget_position, 
            QAngle iangle_offset,
            QAngle idrop_angle, 
            double iharsh_coeff, 
            double icoast_coeff, 
            QTime ibrake_time) {
                return std::make_shared<LookAt>(
                    imax_power, icoast_power, itarget_position,
                    iangle_offset, idrop_angle, iharsh_coeff, icoast_coeff, ibrake_time
                );
            }

        static std::shared_ptr<LookAt> create(
            double imax_power, 
            double icoast_power, 
            Position itarget_position, 
            QAngle iangle_offset,
            QAngle idrop_angle, 
            double iharsh_coeff, 
            double icoast_coeff, 
            QTime ibrake_time,
            QTime itimeout) {
                return std::make_shared<LookAt>(
                    imax_power, icoast_power, itarget_position,
                    iangle_offset, idrop_angle, iharsh_coeff, icoast_coeff, ibrake_time, itimeout
                );
            }

        double progress() override { return 0; }

    private:
        enum class TurnState { FULLPOWER, COAST, BRAKE };

        double max_power;
        double coast_power;
        double harsh_coeff;
        double coast_coeff;
        uint32_t brake_time;
        uint32_t timeout = 0;

        Position start_position;
        Position target_position;
        QAngle angle_goal = 0_deg;
        QAngle angle_offset = 0_deg;
        QAngle drop_angle;

        QAngle angle_difference;
        QAngle target_relative_original;
        QAngle target_relative;
        int spin_direction = 0;
        int brake_start_time = -1;
        TurnState controller_state{TurnState::FULLPOWER};
};

}  // namespace rev
#pragma once
#include <memory>
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/alg/reckless/segment.hh"
#include "rev/api/units/q_angle.hh"
#include "rev/api/units/q_length.hh"
#include "rev/api/units/q_time.hh"

namespace rev {
    class RecklessLookAt : public RecklessSegment {
        public:
            RecklessLookAt(
                double imax_power, 
                double icoast_power, 
                Position itarget_position, 
                QAngle idrop_angle, 
                double iharsh_coeff, 
                double icoast_coeff, 
                QTime ibrake_time);
            
            void init(OdometryState initial_state);

            SegmentStatus step(OdometryState current_state);

            void clean_up();

            std::shared_ptr<RecklessSegment> operator&() {
                return std::make_shared<RecklessLookAt>(*this);
            }

            static std::shared_ptr<RecklessLookAt> create(
                double imax_power, 
                double icoast_power, 
                Position itarget_position, 
                QAngle idrop_angle, 
                double iharsh_coeff, 
                double icoast_coeff, 
                QTime ibrake_time){
                    return std::make_shared<RecklessLookAt>(
                        imax_power,
                        icoast_power,
                        itarget_position,
                        idrop_angle,
                        iharsh_coeff,
                        icoast_coeff,
                        ibrake_time
                    );
                }
        private:
            double max_power;
            double coast_power;
            Position start_position;
            Position target_position;
            QAngle angle_goal = 0_deg;
            QAngle drop_angle;
            double harsh_coeff;
            double coast_coeff;
            uint32_t brake_time;


    };
    using LookAt = RecklessLookAt;
}
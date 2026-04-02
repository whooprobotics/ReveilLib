#pragma once
#include <memory>
#include "rev/api/alg/slipstream/power.hh"
#include "rev/api/alg/slipstream/segment.hh"
#include "rev/api/alg/slipstream/path.hh"
#include "rev/api/alg/slipstream/slipstream.hh"
#include "rev/api/alg/slipstream/mecanum_segment.hh"
#include "rev/api/alg/slipstream/turn_segement.hh"
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
            
            void init(OdometryState initial_state);

            SegmentStatus step(OdometryState current_state);

            void clean_up();

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
                QTime ibrake_time){
                    return std::make_shared<LookAt>(
                        imax_power,
                        icoast_power,
                        itarget_position,
                        idrop_angle,
                        iharsh_coeff,
                        icoast_coeff,
                        ibrake_time
                    );
                }
        double progress() override { return 0; }

        private:
            SlipstreamTurnSegment turn_segment;
            double max_power;
            double coast_power;
            Position start_position;
            Position target_position;
            QAngle angle_goal = 0_deg;
            QAngle drop_angle;
            double harsh_coeff;
            double coast_coeff;
            QTime brake_time;


    };
}
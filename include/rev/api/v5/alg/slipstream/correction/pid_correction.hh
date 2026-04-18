#pragma once

#ifdef PLATFORM_BRAIN

#include <memory>
#include "rev/api/v5/alg/slipstream/correction/holonomic_correction.hh"
#include "rev/api/common/units/q_length.hh"

namespace rev {

    /**
     * @brief Implements pid correction algorithm for holonomic drives
     *
     * PID correction has three terms (proportional, integral, derivative) and corrects
     * movement in 3 directions (forward, lateral, heading)
     * 
     * Error is calculated by finding distance from current position to target position and 
     * rotating this vector by the heading angle 
     *
     * Proportional = Error * proportional coefficient,
     * 
     * Integral = Error Sum * integral coefficient, 
     * 
     * Derivative = Error Rate of Change * derivative coefficient
     * 
     * These are calculated for each direction and summed into the motor powers
     */
    class PIDCorrection : public HolonomicCorrection{

        /**
         * @brief Applies correction to the input powers
         *
         * @param current_state The current OdometryState
         * @param target_state The position being targeted
         * @param start_state The position when the current segment began
         * @param drop_early The distance from target at which segment should end
         * @param powers The input powers from the motion controller
         * @return SlipstreamPower The corrected powers for all four wheels
         */
        SlipstreamPower apply_correction(OdometryState current_state,
                                   Position target_state,
                                   Position start_state,
                                   QLength drop_early,
                                   SlipstreamPower powers) override;

        /**
         * @brief Construct a new PID Correction object
         *
         * @param kp The proportional gain for correction.
         * Higher values provide stronger correction but may cause overshoot
         * @param ki The integral gain for correction.
         * Higher values move error faster to 0 over time but may cause integral "windup" (overshoot and slow return to target)
         * @param kd The derivative gain for correction
         * Higher values limit overshoot but may cause an unstable system (large oscillations)
         * @param imax_forward_error The forward error threshold over which forward correction will be applied
         * @param imax_lateral_error The heading error threshold over which lateral correction will be applied
         * @param imax_heading_error The heading error threshold over which heading correction will be applied
         */
        PIDCorrection(double kp,
                       double ki,
                       double kd,
                       QLength imax_forward_error,
                       QLength imax_lateral_error,
                       QAngle imax_heading_error);

        std::shared_ptr<PIDCorrection> operator&() {
            return std::make_shared<PIDCorrection>(*this);
        }

        private:
            double kp;
            double ki;
            double kd;
            QLength max_forward_error;
            QLength max_lateral_error;
            QAngle max_heading_error;
            double time;
            double integral_sum_forward;
            double integral_sum_lateral;
            double integral_sum_heading;
            double last_forward_error;
            double last_lateral_error;
            double last_heading_error;
    };
}  // namespace rev

#endif
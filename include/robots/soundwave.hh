#pragma once

#ifdef PLATFORM_BRAIN

#include "rev/rev.hh"

namespace soundwave {

void init();

void drive();

void auton();


}  // namespace soundwave

namespace rev {
  
template<>
struct Defaults<PilonsParams> {
  constexpr static double power = 0.65;
  constexpr static QLength drop_early = 0_in;
  constexpr static double k_correction = 2;
  constexpr static QLength max_error = 0.5_in;
  constexpr static QTime harsh = 0.09_s;
  constexpr static QTime coast = 0.3_s;
  constexpr static double coast_power = 0.2;
};

template<>
struct Defaults<TurnSegmentParams> {
  constexpr static double max_power = 0.75;
  constexpr static double coast_power = 0.25;
  constexpr static double harsh = 0.08;
  constexpr static double coast = 0.3;
  constexpr static QTime brake_time = 0.2_s;
  constexpr static QTime timeout = 0.0_s;
};

template<>
struct Defaults<LookAtParams> {
  constexpr static double max_power = 0.75;
  constexpr static double coast_power = 0.25;
  constexpr static double harsh = 0.08;
  constexpr static double coast = 0.3;
  constexpr static QAngle angle_offset = 0.0_deg;
  constexpr static QAngle drop_angle = 0.0_deg;
  constexpr static QTime brake_time = 0.2_s;
  constexpr static QTime timeout = 0.0_s;
};
}  // namespace rev

#endif
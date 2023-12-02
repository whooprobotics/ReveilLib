#include "rev/api/alg/drive/correction/pilons_correction.hh"
#include <cmath>
#include <tuple>
#include "rev/api/units/q_angle.hh"
#include "rev/api/units/q_length.hh"

rev::PilonsCorrection::PilonsCorrection(double ikCorrection, QLength imaxError)
    : k_correction(ikCorrection), max_error(imaxError) {}

std::tuple<double, double> rev::PilonsCorrection::apply_correction(
    rev::OdometryState current_state,
    rev::Position target_state,
    Position start_state,
    QLength drop_early,
    std::tuple<double, double> powers) {
  // Dot product shit
  Number facingx = cos(current_state.pos.facing);
  Number facingy = sin(current_state.pos.facing);

  QLength tarposx = target_state.x - current_state.pos.x;
  QLength tarposy = target_state.y - current_state.pos.y;
  QLength tarposabs = sqrt(tarposx * tarposx + tarposy * tarposy);

  Number cosang = (tarposx * facingx + tarposy * facingy) / tarposabs;

  QLength err_x = tarposy * facingx - tarposx * facingy;

  QAngle ang = acos(cosang);

  if (err_x < 0_m)
    ang = -ang;

  char sgn_power = (std::get<0>(powers) + std::get<1>(powers)) >= 0 ? 1 : -1;

  double correction = abs(err_x) > abs(max_error)
                          ? k_correction * ang.convert(radian) * sgn_power
                          : 0.0;

  if (correction > 0)
    return std::make_tuple(std::get<0>(powers),
                           std::get<1>(powers) * exp(-correction));
  else if (correction < 0)
    return std::make_tuple(std::get<0>(powers) * exp(correction),
                           std::get<1>(powers));
  else
    return powers;
}
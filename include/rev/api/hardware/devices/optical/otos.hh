#pragma once
#include "rev/api/alg/odometry/odometry.hh"
#include "rev/api/units/all_units.hh"
#include <string>
#include <vector>
#include <sstream>

namespace rev {

class OTOS {
  public:
    OTOS();
    // Return position values
    double get_x();
    double get_y();
    double get_h();
    // Get new values from sensor
    void update();

  private:
    double x;
    double y;
    double h;

    std::string input;
    std::string token;
    std::vector<std::string> tokens;
    std::stringstream ss;

};
} // namespace rev


#include "rev/util/mathutil.hh"

namespace rev{

QLength calculate_radius(PointVector first_point, Position current_pos, PointVector next_point) {
    // Define the three points
    PointVector A = {first_point.x, first_point.y};
    PointVector B = {current_pos.x, current_pos.y};
    PointVector C = {next_point.x, next_point.y};

    // Ok so curvature = 2*area / (product of side lengths)

    // Calculate side lengths
    QLength AB = sqrt((B.x - A.x) * (B.x - A.x) + (B.y - A.y) * (B.y - A.y));
    QLength BC = sqrt((C.x - B.x) * (C.x - B.x) + (C.y - B.y) * (C.y - B.y));
    QLength CA = sqrt((A.x - C.x) * (A.x - C.x) + (A.y - C.y) * (A.y - C.y));

    // If any side lengths are 0, then curvature has a division by 0 so is undef, and radius would be 0
    if (AB == 0_ft || BC == 0_ft || CA == 0_ft) return 0_ft;

    // Calculate the area of the triangle using the determinant method
    QArea area = 0.5 * (A.x * (B.y - C.y) + B.x * (C.y - A.y) + C.x * (A.y - B.y));

    // Calculate curvature (kappa = 2 * area / (AB * BC * CA))
    RQuantity curvature = (2 * area) / (AB * BC * CA); // curvature is an inverse length
    RQuantity zero_inverse_length = 0_in / (1_in * 1_in); // This is really sketchy lmao

    // if curvature is 0, basically a straight line so put very high number for radius so the 'circular' path is basically straight
    if (curvature == zero_inverse_length) return 1000_ft;

    return 1/curvature;
}

Number calculate_inside_ratio(QLength chassis_width, QLength arc_radius){
  QLength outside_wheels = arc_radius + (chassis_width/2.0);
  QLength inside_wheels = arc_radius - (chassis_width/2.0);

  return inside_wheels / outside_wheels;
}

}
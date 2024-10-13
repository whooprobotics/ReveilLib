#include "rev/util/mathutil.hh"

namespace rev{

std::tuple<QLength, bool> calculate_radius(Position first_point, PointVector next_coords){
  QAngle current_heading = first_point.theta;
  PointVector first_coords = {first_point.x, first_point.y};
  PointVector midpoint_coords = {(first_coords.x + next_coords.x)/2.0, (first_coords.y + next_coords.y)/2.0};

  Number coords_heading_slope = (next_coords.y - first_coords.y) / (next_coords.x - first_coords.x);
  Number perp_bisector_slope = (-1.0)/coords_heading_slope;
  Number perp_heading = tan(current_heading + 90_deg);
  
  PointVector center;
  center.x = (perp_heading * first_coords.x - perp_bisector_slope * midpoint_coords.x + midpoint_coords.y - first_coords.y)
             / (perp_heading - perp_bisector_slope);
  center.y = perp_bisector_slope * (center.x - midpoint_coords.x) + midpoint_coords.y;
  
  QLength radius = sqrt((center.x - first_coords.x)*(center.x - first_coords.x) + (center.y - first_coords.y)*(center.y - first_coords.y));

  QAngle angle_to_center = atan2(center.y - first_coords.y, center.x - first_coords.x);
  bool is_left_closer = (current_heading - angle_to_center < 0_deg);
  
  return std::make_tuple(radius, is_left_closer);

}

Number calculate_inside_ratio(QLength chassis_width, QLength arc_radius){
  QLength outside_wheels = arc_radius + (chassis_width/2.0);
  QLength inside_wheels = arc_radius - (chassis_width/2.0);

  return inside_wheels / outside_wheels;
}

}